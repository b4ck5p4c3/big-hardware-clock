/**
 * The MIT License (MIT)
 * Copyright (c) 2015 by Fabrice Weinberg
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "NTPClient.h"
#include <math.h>
#include <limits.h>
#include "assume.h"

#define ARRAY_LEN(x) (sizeof(x) / sizeof((x)[0]))
#define ARRAY_END(x) (x + ARRAY_LEN(x))
#define PRINT(x) do { Serial.print( #x ": " ); Serial.println(x); } while (0)
#define PRINT2(x, fmt) do { Serial.print( #x ": " ); Serial.println((x), fmt); } while (0)

// System Precision is (-16) as an upper bound for the following values:
//   - 8us stands for -16.93, that's precision of Arduino Lilypad micros()
//   - 8.62us stands for -16.82, that's precision of lfplo2us() and us2lfplo()
//   - -17 stands for 7.63us, that's -16.93 rounded
#define S_PRECISION (-16)
#define S_PRECISIONMICROS 9

// Resolution of micros() is 2 on ATmega328p, precision (in NTP sense) 4.  Timeout of 2s (BTIME)
// needs 21 bit to be represented in micros, but 32us granularity is fine for delay measurement.
#define MICROS16_SHR 5

#define LOG2D(a) ((a) < 0 ? 1. / (1L << -(a)) : 1L << (a)) // poll, etc. It's rather EXP2D :-)

static const uint8_t VerModeMask = 0b00111111;
static const uint8_t Ver4Client = 0b00100011;
static const uint8_t Ver4Server = 0b00100100;

struct NtpPacket {
  uint8_t LeapVerMode;
  uint8_t Stratum;
  uint8_t Poll;
  uint8_t Precision;
  uint32_t RootDelay;
  uint32_t RootDispersion;
  uint32_t RefID;
  uint64_t RefTime;
  uint64_t Org;
  uint64_t Rec;
  uint64_t Xmt;

  uint8_t VerMode() const { return LeapVerMode & VerModeMask; }
  uint8_t Leap() const { return (LeapVerMode >> 6) & 3; }
  uint32_t RootDistFP() const;  // not root_dist()
};
#define REF_OFFSET 16
#define ORG_OFFSET 24
#define REC_OFFSET 32
#define XMT_OFFSET 40
#define NTP_PACKET_SIZE 48
static_assert(sizeof(NtpPacket) == NTP_PACKET_SIZE);

// This SNTP implementation always steps, so `c.offset`, aka "current clock offset" is always zero.
// Note, it's different from `c.last` also known as "previous offset".
static const double clockOffset = 0;
static const double FRAC = 4294967296.;  // 2^32 as a double

#define PORT 123        // NTP port number
#define BTIME 2         // burst interval (s), also default guard interval
#define MINDISP .01     // % minimum dispersion (s) // FIXME: or 0.005 ?
#define MAXDISP 16      // maximum dispersion (s)
#define MAXDIST 1       // % distance threshold (s)
#define NOSYNC 0x3      // leap unsync
#define KODSTRAT 0      // Stratum=0 are called Kiss-o'-Death (KoD) packets
#define MAXSTRAT 16     // maximum stratum number
#define MAXPOLL 17      // 36h, maximum poll exponent
#define MINPOLL_NTP 4   // 16s, minimum poll exponent for NTPv4, RFC5905
#define MINPOLL_SNTP 6  // 64s, minimum poll exponent for SNTPv4, RFC4330 Section 5
#define PHI 15e-6       // frequency tolerance (15 ppm)
#define PHIe6 15        // frequency tolerance (15 ppm)

static const uint32_t MaxDispersionFP = UINT32_C(MAXDISP) << 16;

// Local clock process return codes
#define IGNORE 0  // ignore
#define SLEW 1    // slew adjustment
#define STEP 2    // step adjustment
#define PANIC 3   // panic - no adjustment

// Clock state definitions
#define NCLK 0 // no RTC clock to bootstrap from, 1st stample moves from NCLK to NSET|FSET
#define NSET 1 // clock never set
#define FSET 2 // frequency set from file
#define SPIK 3 // spike detected
#define FREQ 4 // frequency (measurement) mode
#define SYNC 5 // clock synchronized

// NTPv3 defines CLOCK.MAX (STEPT) as ±128ms for Crystal clock and ±512ms for Mains clock:
// https://www.eecis.udel.edu/~mills/database/rfc/rfc1305/rfc1305b.pdf
// See https://www.eecis.udel.edu/~mills/ntp/html/clock.html regarding WATCH=300
#define STEPT .125         // step threshold (s)
#define WATCH 900          // stepout threshold (s), 300s for NTP-4.2.8p18
#define PANICT 1000        // panic threshold (s)
#define AVG 8              // parameter averaging constant, 4 in RFC5905, 8 in NTP-4.2.8p18
#define ALLAN_XPT 11       // Allan intercept (log2(ALLAN))
#define LIMIT 30           // poll-adjust threshold
#define PGATE 4            // poll-adjust gate
#define SGATE 3            // spike gate (clock filter)

#define MAXFREQ (NTPCLIENT_MAXFREQ * 1e-6)  // NTPCLIENT_MAXFREQ is not normally changed

#define MINPOLL MINPOLL_SNTP  // That's not NTP, right?
#define JAN_1970 2208988800UL // 1970 - 1900 in seconds

// Terms of Service
// [...]
// 4. End-User agrees that he or she will not:
// [...]
// (b) Request time from the Services more than once every thirty (30) minutes (ideally at even
// longer intervals), if using SNTP.  -- https://www.ntppool.org/tos.html
#define NTPPOOL_MINPOLL 11
#define NTPPOOL_DOMAIN "pool.ntp.org"

#define IS_POW2(x) (((x) & (x - 1)) == 0)

static bool is_big_endian(void) {
#if defined(__BIG_ENDIAN__)
  return true;
#elif defined(__BYTE_ORDER__) && defined(__ORDER_BIG_ENDIAN__)
  return __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
#else
  const uint32_t i = 0x01020304;
  const uint32_t* pi = &i;
  const uint8_t* plsb = pi;
  return *plsb == 0x01;
#endif
}
static uint64_t ntoh64(uint64_t x) {
  // TODO: if not defined __builtin_bswap64
  return is_big_endian() ? x : __builtin_bswap64(x);
}
static uint32_t ntoh32(uint32_t x) {
  return is_big_endian() ? x : __builtin_bswap32(x);
}

static int clzul(unsigned long x) {
  static_assert(sizeof(x) == 4);
  if (!x) return 32;
  int n = 0;
  if (!(x & 0xFFFF0000UL)) { n += 16; x <<= 16; }
  if (!(x & 0xFF000000UL)) { n += 8; x <<= 8; }
  if (!(x & 0xF0000000UL)) { n += 4; x <<= 4; }
  if (!(x & 0xC0000000UL)) { n += 2; x <<= 2; }
  if (!(x & 0x80000000UL)) { n++; }
  return n;
}

static int log2ul(unsigned long x) { return 32 - 1 - clzul(x); }

// Integer approximations lfplo2us() and us2lfplo() allow to avoid floating point arithmetics while
// introducing average error of 4us and max error of 8.62us.  That's absolutely okay given limited
// resolution of micros() on some Arduino boards: 4us on Nano, 8us on Lilypad.
static unsigned long lfplo2us(uint32_t lfplo) {
  // 2^32 / 1e6 = 4294.967296 ≈4295
  return lfplo / UINT32_C(4295);
}

static uint32_t us2lfplo(unsigned long us) {
  ASSUME(us <= 999999UL);
  // 4295 * 999992 == 0xfffff988
  if (us <= 999992UL)
    return us * UINT32_C(4295);
  // 999992…999999 ~ 0xF4238…0xF423F
  return UINT32_C(0xFFFFF988) + UINT32_C(207) * (us & 7);
}

static uint64_t make_lfp48(uint32_t hi, uint16_t lo) {
  return (uint64_t(hi) << 32) | (uint32_t(lo) << 16);
}

static unsigned char millisToPollExponent(unsigned long updateInterval) {
  updateInterval /= 1000;
  unsigned char pollExponent = log2ul(updateInterval);
  if (!IS_POW2(updateInterval))
    pollExponent++;  // 3600_000ms should map to 64s, not 32s ;-)
  if (pollExponent < MINPOLL)
    pollExponent = MINPOLL;
  if (pollExponent > MAXPOLL)
    pollExponent = MAXPOLL;
  return pollExponent;
}

// Modern compiler is capable of evaluating that at compile and/or link time.
static bool strEndsWith(const char* haystack, const char* needle) {
  return strlen(haystack) >= strlen(needle) &&
         strstr(haystack, needle) == haystack + strlen(haystack) - strlen(needle);
}

static uint16_t randomEphemeralPort(void) {
  // Port is randomized per RFC 6056 (Recommendations for Transport-Protocol Port Randomization).
  // Section 3.2 (Ephemeral Port Number Range) suggests to use the full range.
  // Port it persisted while talking to the same ServerIP to reduce Effects on Path Selection per
  // RFC 9109 (NTPv4: Port Randomization).  Real-world ECMP routes might be dozens of milliseconds
  // different and lead to unnecessary jitter.  And low jitter is a key to higher poll interval.
  // Every random(void) call is ≈51 micros and random(NN) is ≈90 micros on Arduino Uno.
  // So that's one sample + fastrange() instead of random(1024, 65535 + 1);
  const uint16_t low = 1024, high = 65535;
  const unsigned long rl = random();
  const uint16_t r16 = rl;
  if (r16 >= low)
    return r16;
  const uint32_t rFolded = uint16_t(rl ^ (rl >> 16));
  const uint16_t r16hi = (rFolded * (high - low)) >> 16;
  return low + r16hi;  // it's biased, but is guaranteed to terminate fast unlike sampling loop
}

static uint64_t micros63(void) {
  // We save 4 bytes using the MSB of `high` to store MSB of `last` as we don't need full value
  // to detect overflow. micros63() overflows in 292 millennia (micros47() takes just 4.5 years).
  static uint32_t high = 0;
  const uint32_t now = micros();  // micros() overflows every 1h11m
  const uint8_t oldMSB = (high >> 24) & 0x80;
  const uint8_t nowMSB = (now >> 24) & 0x80;
  if (nowMSB < oldMSB)
    high++;
  high &= UINT32_C(0x7FFFFFFF);
  const uint64_t ret = (uint64_t(high) << 32) | now;
  high |= now & UINT32_C(0x80000000);
  return ret;
}

uint32_t NtpPacket::RootDistFP(void) const {
  return (ntoh32(RootDelay) >> 1) + ntoh32(RootDispersion);
}

// RFC 5424 (The Syslog Protocol) defines maximum value of 2147483647 and says "If that value
// is reached, the next message MUST be sent with a sequenceId of 1".  Overflow is impossible
// as the syslog message is emitted only once per a valid reply (per 2 seconds).
#ifdef NTPCLIENT_SYSLOG
uint32_t NTPClient::sequenceId = 0;
#endif

NTPClient::NTPClient(UDP& udp, long timeOffset)
  : _udp(udp)
{
  ctor(timeOffset);
  // default updateInterval is 60s for NTPClient 3.x.x, so MINPOLL_SNTP of 6 suits well.
  this->_pollExponent = minpoll();
}

NTPClient::NTPClient(UDP& udp, const char* poolServerName, long timeOffset, unsigned long updateInterval)
  : _udp(udp)
{
  ctor(timeOffset);
  this->_poolServerName = poolServerName;
  this->_doResolve      = true;
  this->_pollExponent   = millisToPollExponent(updateInterval);
  if (isPollingNtppool())
    this->_pollExponent = max(_pollExponent, (uint8_t)NTPPOOL_MINPOLL);

}

NTPClient::NTPClient(UDP& udp, IPAddress poolServerIP, long timeOffset, unsigned long updateInterval)
  : _udp(udp)
{
  ctor(timeOffset);
  this->_poolServerIP   = poolServerIP;
  this->_poolServerName = NULL;
  this->_pollExponent   = millisToPollExponent(updateInterval);
}

void NTPClient::ctor(long timeOffset) {
  _timeOffset = timeOffset;
  _keepPortOnNewIP = false;
  _udpSetup = false;
  _doResolve = false;
  _waitingForReply = false;
  _clockState = NCLK;
  _freq = 0.0;  // TODO: add flag for FSET
  _lastMicros = 0;
  _lastUnix = 0;
  _lastOffset = 0;
#ifdef NTPCLIENT_WANDER
  _wander = 0;
#endif
#ifdef NTPCLIENT_SYSLOG
  _pid = random();
#endif
  _jiggleCount = 0;
  clearAssociation();
}

void NTPClient::clearAssociation() {
  for (Sample *s = _filter; s != ARRAY_END(_filter); s++) {
    s->unixHigh = 0;
    s->unixLFP16 = 0;
    s->delayMicros16 = UINT16_MAX;
    s->offset = 0;
  }
  _filterNextIndex = 0;
  _reach = 0;
  _pollExponent = minpoll();
  // NTP-4.2.8 resets `sys_jitter` on STEP, while RFC5905 does not reset jitter
  _clockJitter = LOG2D(S_PRECISION);
}

bool NTPClient::isPollingNtppool() const {
  // TODO: close `pool.ntp.org.` gap of adding a trailing dot
  return _poolServerName && strEndsWith(_poolServerName, NTPPOOL_DOMAIN);
}

unsigned char NTPClient::minpoll() const {
  return isPollingNtppool() ? NTPPOOL_MINPOLL : MINPOLL;
}

void NTPClient::begin() {
  end();
  if (!this->_keepPortOnNewIP)
    this->_port = randomEphemeralPort();
  if (this->_udp.begin(this->_port) != 0)
    this->_udpSetup = true;
}

void NTPClient::begin(unsigned int port) {
  this->_port = port;
  this->_keepPortOnNewIP = true;
  begin();
}

bool NTPClient::forceUpdate() {
  if (!_waitingForReply) {
    if (sendRequest()) {
      _waitingForReply = true;
    } else {
      return false;
    }
  }
  return update();
}

bool NTPClient::maintain() {
  if (!_waitingForReply) {
    const unsigned long tillPoll = _nextPollMillis - millis();  // >LONG_MAX on overflow
    if (tillPoll > LONG_MAX || _clockState == NCLK) {
      if (sendRequest())
        _waitingForReply = true;
    }
    return false;
  } else {
    return checkForReply();
  }
}

bool NTPClient::sendRequest() {
  if (!this->_udpSetup)
    this->begin();
  if (!this->_udpSetup)
    return false;

  // flush any existing packets
  while(this->_udp.parsePacket() != 0)
    this->_udp.flush();

  _reach <<= 1;
  // random() is a 31-bit generator, so two bits are quite deterministic. Anyway, it's not CSPRNG.
  // Also random() is somewhat slow, so two bits matter less than one extra random() call.
  _xmt[0] = random();
  _xmt[1] = random();
  this->sendNTPPacket((unsigned char*)_xmt);
  return true;
}

bool NTPClient::checkForReply() {
  int cb = 0;
  do {
    cb = this->_udp.parsePacket();
    // TODO: check if parsePacket() discards the current packet on mismatching size?
  } while (cb && cb != NTP_PACKET_SIZE);
  const uint64_t t4 = micros63();
  const unsigned long t4low = t4;
  const unsigned long dt41 = t4low - _t1;
  if (!cb) {
    // NTPClient-3.2.1 had default timeout of 1000ms, but 2s is picked here as it is based on BTIME.
    // Reference implementation does not respond to the same node more often than once in BTIME.
    // ntpdate from the reference implementation has 2s as a timeout as well.
    // Timeout of 2s also correlates well with MAXDIST of 1s and root_dist() calculation.
    // So it's a good timeout from update() and forceUpdate() standpoint.
    // See also "Minimum Headway Time" in https://www.eecis.udel.edu/~mills/ntp/html/rate.html
    if (dt41 > BTIME * 1000UL * 1000UL)
      scheduleNextPoll();
    return false;
  }

  union {
    byte pkt[NTP_PACKET_SIZE];
    NtpPacket ntp;
  } u;
  static_assert((void*)&u.pkt == (void*)&u.ntp);
  byte* const pkt = u.pkt;
  this->_udp.read(pkt, NTP_PACKET_SIZE);

  // Using `memcmp() > 0` for ordering does the wrong thing for some seconds durring NTP rollover
  // in 2036.  We'll be good as soon both RefTime and Xmt overflow, so it's not a big deal.
  // memcmp() should save few bits on ATmega328p compared to true uint64_t math.
  if (memcmp(pkt + ORG_OFFSET, _xmt, 8) != 0 ||
      (!_doResolve && _udp.remoteIP() != _poolServerIP) ||
      _udp.remotePort() != PORT ||
      u.ntp.VerMode() != Ver4Server || u.ntp.Stratum > MAXSTRAT ||
      u.ntp.RootDistFP() >= MaxDispersionFP ||
      memcmp(pkt + REC_OFFSET, pkt + XMT_OFFSET, 8) > 0 ||
      memcmp(pkt + REF_OFFSET, pkt + XMT_OFFSET, 8) > 0)
    return false; // mismatching cookie and/or malformed packet

  // u.ntp.RootDistFP > 16s is malformed
  // u.ntp.RootDistFP > 1.5s is unfit()
  // TODO: u.ntp.Stratum || u.ntp.RootDistFP might also be the reason to _doResolve

  // Now, we're confident that we got well-formed Reply from The Server.
  const uint64_t rec = ntoh64(u.ntp.Rec);
  const uint64_t xmt = ntoh64(u.ntp.Xmt);
  const uint64_t srvDtLFP = xmt - rec;
  const uint32_t srvDtSec = srvDtLFP >> 32;
  const uint32_t srvDtLow = srvDtLFP;
  const uint64_t srvMidpLFP = rec + (srvDtLFP >> 1);  // in LFP, NTP-Long-Fixed-Point
  const uint32_t srvMidpSec = srvMidpLFP >> 32;
  const uint32_t srvMidpLow = srvMidpLFP;
  const int64_t ourMidpMicros = t4 - (dt41 >> 1);    // in micros63()
  const uint64_t ourMidpUnixLFP = unixLFPAt(ourMidpMicros);
  const uint32_t srvMidpUnix = srvMidpSec - JAN_1970;
  const uint64_t srvMidpUnixLFP = uint64_t(srvMidpUnix) << 32 | srvMidpLow;
  const int64_t offsetLFP = srvMidpUnixLFP - ourMidpUnixLFP;
  const double offset = offsetLFP / FRAC;

  bool clockUpdated = false;
  if (u.ntp.Stratum == KODSTRAT) {
    // An SNTP client should stop sending to a particular server if that server returns a reply with
    // a Stratum field of 0 (kiss-o'-death packet), regardless of kiss code, and an alternate server
    // is available.  If no alternate server is available, the client should retransmit using an
    // exponential backoff algorithm. -- RFC 4330: SNTPv4
    //
    // Unfortunately, UDP class does not provide DNS API, so we don't really know if we have
    // an alternate server or not.  So, we have to resort to doing next query with DNS resolution
    // to, maybe, get a new IP, and increasing poll interval at the same time.
    _pollExponent = min(_pollExponent + 1, MAXPOLL);
    _doResolve = true;
    // TODO: verify that actual KoD senders pass malformed packet filter and reach this code
  } else if (srvDtSec == 0 && u.ntp.Leap() != NOSYNC && u.ntp.Stratum != MAXSTRAT) {
    if (_doResolve) {
      _doResolve = false;
      const IPAddress remoteIP(_udp.remoteIP());
      if (_poolServerIP != remoteIP) {
        _pollExponent = minpoll();  // new association
        _poolServerIP = remoteIP;
      }
    }

    _reach |= 1;
    if (_clockState != NCLK) {
      // Offset = theta = T(B) - T(A) = [(T2-T1) + (T3-T4)]/2 = [(T2+T3) - (T1+T4)]/2
      // Delay  = delta = T(ABA) = (T4-T1) - (T3-T2)
      //
      // The delay calculation is a special case.  In cases where the server and client clocks
      // are running at different rates and with very fast networks, the delay can appear negative.
      // In order to avoid violating the Principle of Least Astonishment, the delay is clamped
      // not less than the system precision.
      //
      // Dispersion is not calculated as it's not very useful in a single-peer case: it's used
      // for peer ranking, for sys.rootdisp (that's not exposeed unless we speak NTP as a server)
      // and as a constant term in root_dist(peer) calculation.
      const int32_t srvDtMicros = lfplo2us(srvDtLow);
      const uint32_t delayMicros = max(int32_t(dt41) - srvDtMicros, S_PRECISIONMICROS);

      clockUpdated = clockFilter(srvMidpUnixLFP, offset, delayMicros);
    } else {
      // Arduino boards usually have no RTC, so the very first sample has truly meaningless offset,
      // so the 1st sample does not go through clock_filter().
      const uint32_t srvMicros = lfplo2us(srvMidpLow);
      _lastUnix = srvMidpUnix;
      _lastUnixLFP16 = srvMidpLow >> 16;
      _lastMicros = ourMidpMicros - srvMicros;  // to avoid fractional part of _lastUnix
      // clearAssociation() was called by ctor()
      _clockState = _freq == 0.0 ? NSET : FSET;
      clockUpdated = true;
    }
  } else {
    // TODO: should we update _pollExponent ?
    _doResolve = true;
  }
  scheduleNextPoll();

  const uint32_t syslogIPu32(
#ifdef NTPCLIENT_SYSLOG
      ntoh32(NTPCLIENT_SYSLOG)
#else
      0
#endif
  );
  if (syslogIPu32) {
#ifndef NTPCLIENT_SYSLOG
    // make compiler less unhappy about undefined variables
    const unsigned long _pid = 0;
    uint32_t sequenceId = 0;
#endif
    // No timestamp on the wire to avoid calendar computation for YYYY-DD-MM part.
    // sysUpTime is hundredths of a second since boot, it overflows in 497 days.
    const unsigned long uptimeCentis = t4 / 10000;
    const char stateNames[6][5] = {"NCLK", "NSET", "FSET", "SPIK", "FREQ", "SYNC"};

    // The message should be under 340 bytes. It depends on MAXPOLL, NSTAGE, BTIME and MAXFREQ.
    // 64338 is B4CKSP4CE Hackerspace PEN (Private Enterprise Numbers) at IANA.
    char msg[344];
    int sz = snprintf(
        msg, sizeof(msg),
        "<101>1 - pogo NTPClient %08lx rec - [meta sysUpTime=\"%lu\" sequenceId=\"%lu\"]"
        "[SNTP@64338 ip=\"%08lx\" pktMIDP=\"%08lx.%08lx\" clkMIDP=\"0x%lx%08lx\" sta=\"%s\" "
        "poll=\"%u\" reach=\"%o\" rtt=\"%ld\" pktOff=\"%+ld\" clkOff=\"%+ld\" "
        "peerJtr=\"%ld\" clkJtr=\"%ld\" freq=\"%+ld\""
#ifdef NTPCLIENT_WANDER
        " wndr=\"%ld\""
#endif
        "]",
        _pid, uptimeCentis, ++sequenceId, ntoh32(_poolServerIP), srvMidpSec, srvMidpLow,
        (unsigned long)(ourMidpMicros >> 32), (unsigned long)(ourMidpMicros),
        stateNames[_clockState], _pollExponent, _reach, dt41, lround(offset * 1e6),
        lround(_lastOffset * 1e6), lround(peerJitter(bestSample(srvMidpUnix)) * 1e6),
        lround(_clockJitter * 1e6), lround(_freq * 1e9)
#ifdef NTPCLIENT_WANDER
        , lround(_wander * 1e9)
#endif
        );
    if ((int)sizeof(msg) < sz)
      sz = sizeof(msg);

    const unsigned syslogPort = 514, discardPort = 9;
    _udp.beginPacket(IPAddress(syslogIPu32), syslogPort);
    _udp.write((uint8_t*)msg, sz);
    _udp.endPacket();

    // Pre-heating ARP cache after _udp socket being reused to send a syslog message.  That's needed
    // on ATmega328p, but it's unclear if ESP32 needs it.  Anyway, syslog is a _debugging_ feature,
    // so it's okay...  XXX: W5100.writeSnTTL(_sock, ttl); ?..
    _udp.beginPacket(_poolServerIP, discardPort);
    _udp.write(0xA5);
    _udp.endPacket();
  }
  return clockUpdated;
}

uint32_t NTPClient::Sample::distance(uint32_t unixNow) const {
  if (delayMicros16 == UINT16_MAX)
    return MAXDISP * UINT32_C(1000000);
  const uint32_t dtSec = unixNow - unixHigh;
  const uint32_t delayMicros = ((uint32_t)delayMicros16) << MICROS16_SHR;
  if (dtSec > LOG2D(ALLAN_XPT)) {
    // Peer precision and S_PRECISIONMICROS are ignored here.  Both are constant and mostly
    // irrelevant as sample dispersion terms given their value being under 10us in 99% of cases.
    return delayMicros + dtSec * PHIe6;
  } else {
    return delayMicros;
  }
  // NTP-4.2.8p18 says: "If the dispersion is greater than the maximum dispersion, clamp
  // the distance at that value".  However. delayMicros is limited to 2s by BTIME-based
  // timeout and dtSec is limited to (1<<MAXPOLL)*(NSTAGE-1). So, the sum is limited to 15.8s.
  // So, clamping to MAXDISP of 16s is important only as a safeguard against coding mistake.
}

const NTPClient::Sample* NTPClient::bestSample(uint32_t unixNow) const {
  // Since samples become increasingly uncorrelated beyond the Allan intercept, only under
  // exceptional cases will an older sample be used.  Therefore, the distance list uses a compound
  // metric.  If the time since the last update is less than the Allan intercept use the delay;
  // otherwise, use the sum of the delay and dispersion.
  const Sample *best = _filter;
  uint32_t bestDistance = best->distance(unixNow);
  for (const Sample *s = _filter + 1; s != _filter + NSTAGE; s++) {
    const uint32_t dst = s->distance(unixNow);
    if (dst < bestDistance || dst == bestDistance && best->unixHigh < s->unixHigh) {
      best = s;
      bestDistance = dst;
    }
  }
  return best;
}

double NTPClient::peerJitter(const Sample *best) const {
  double peerJitter = 0;
  int valid = 0;
  for (const Sample *s = _filter; s != _filter + NSTAGE; s++)
    if (s != best && s->delayMicros16 != UINT16_MAX) {
      peerJitter += sq(s->offset - best->offset);
      valid++;
    }
  peerJitter = max(valid ? sqrt(peerJitter / valid) : 0, LOG2D(S_PRECISION));
  return peerJitter;
}

// clock_filter() - select the best from the latest eight delay/offset samples.
bool NTPClient::clockFilter(uint64_t unixLFP, double offset, uint32_t delayMicros) {
  // The clock filter contents consist of eight tuples (offset, delay, time). Without dispersion.
  // Reference implementation uses shift register, that's a ring buffer.
  Sample& next = _filter[_filterNextIndex++];
  next.unixHigh = unixLFP >> 32;
  next.unixLFP16 = uint16_t(unixLFP >> 16);
  next.delayMicros16 = (delayMicros + (UINT32_C(1) << MICROS16_SHR) - 1) >> MICROS16_SHR;
  next.offset = offset;

  // XXX: local clock should rather be used here instead of next.unixHigh
  const Sample *best = bestSample(next.unixHigh);

  // Prime directive: use a sample only once and never a sample older than the latest one.
  // RFC says "anything goes before first synchronized", but NOSYNC is handled at NSET stage.
  if (best->unixHigh <= _lastUnix)  // uint32_t Unix time is good till 2106, that's okay.
    return false;

  // Popcorn spike suppressor.  Compare the difference between the last and current offsets
  // to the current jitter.  If greater than SGATE (3) and if the interval since the last offset
  // is less than twice the system poll interval, dump the spike.
  if (fabs(best->offset - _lastOffset) > SGATE * peerJitter(best) &&
      (best->unixHigh - _lastUnix) < (UINT32_C(2) << _pollExponent))
    return false;

  if (best != &next) {
    unixLFP = make_lfp48(best->unixHigh, best->unixLFP16);
    offset = best->offset;
  }
  return clockUpdate(unixLFP, offset);
}

bool NTPClient::clockUpdate(uint64_t unixLFP, double offset) {
  switch (localClock(unixLFP, offset)) {
    // The offset is too large and probably bogus.  Complain to the system log and order
    // the operator to set the clock manually within PANIC range.  The reference implementation
    // includes a command line option to disable this check and to change the panic threshold
    // from the default 1000 s as required.
    case PANIC:
      _doResolve = true;
      // XXX: _clockState = NSET ?...
      return false;

    // The offset is more than the step threshold (0.125 s by default).  After a step, all
    // associations now have inconsistent time values, so they are reset and started fresh.
    // The step threshold can be changed in the reference implementation in order to lessen
    // the chance the clock might be stepped backwards.  However, there may be serious consequences,
    // as noted in the white papers at the NTP project site.
    case STEP:
      clearAssociation();
      // s.stratum = MAXSTRAT;
      return true;

    // The offset was less than the step threshold, which is the normal case.  Update the system
    // variables from the peer variables.  The lower clamp on the dispersion increase is to avoid
    // timing loops and clockhopping when highly precise sources are in play.  The clamp can be
    // changed from the default .01 s in the reference implementation.
    case SLEW:
      return true;

    // Some samples are discarded while, for instance, a direct frequency measurement is being made.
    case IGNORE:
      return false;
  }
  return false;  // FIXME: unreachable
}

unsigned char NTPClient::localClock(uint64_t unixLFP, double offset) {
  // Following times are used in RFC's local_clock():
  //  - p.t -- last sample out of peer's clock_filter()
  //  - s.t -- last system clock update time
  //  - c.t -- process time, "now"
  double etemp, dtemp;

  // If the offset is too large, give up and go home.
  if (fabs(offset) > PANICT)
    return PANIC;

  // Clock state machine transition function.  This is where the action is and defines
  // how the system reacts to large time and frequency errors.  There are two main regimes:
  // when the offset exceeds the step threshold and when it does not.
  int rval = SLEW;
  const double mu = (unixLFP - make_lfp48(_lastUnix, _lastUnixLFP16)) / FRAC;  // interval since last update
  double freq = 0;  // frequency error
  if (fabs(offset) > STEPT) {
    switch (_clockState) {
      // In SYNC state, we ignore the first outlier and switch to SPIK state.
      case SYNC:
        _clockState = SPIK;
        return SLEW;

      // In FREQ state, we ignore outliers and inliers.  At the first outlier after the stepout
      // threshold, compute the apparent frequency correction and step the time.
      case FREQ:
        if (mu < WATCH)
          return IGNORE;

        freq = (offset - clockOffset) / mu;
        // fall through

      // In SPIK state, we ignore succeeding outliers until either an inlier is found or the stepout
      // threshold is exceeded.
      case SPIK:
        if (mu < WATCH)
          return IGNORE;
        // fall through

      // We get here by default in NSET and FSET states and from above in FREQ state.
      // Step the time and clamp down the poll interval.
      //
      // In NSET state, an initial frequency correction is not available, usually because
      // the frequency file has not yet been written.  Since the time is outside the capture range,
      // the clock is stepped.  The frequency will be set directly following the stepout interval.
      //
      // In FSET state, the initial frequency has been set from the frequency file.  Since the time
      // is outside the capture range, the clock is stepped immediately, rather than after the stepout
      // interval.  Guys get nervous if it takes 17 minutes to set the clock for the first time.
      //
      // In SPIK state, the stepout threshold has expired and the phase is still above the step
      // threshold.  Note that a single spike greater than the step threshold is always suppressed,
      // even at the longer poll intervals.
      default:
        // This is the kernel set time function, usually implemented by the Unix settimeofday()
        // system call.
        step_time(offset);
        _jiggleCount = 0;
        _clockJitter = LOG2D(S_PRECISION);  // it's also reset on STEP, but it's used below.
        _pollExponent = minpoll();
        rval = STEP;
        if (_clockState == NSET) {
          rstclock(FREQ, unixLFP, 0);
          return STEP;
        }
    }
    rstclock(SYNC, unixLFP, 0);
    // XXX: NTP-4.2.8 still updates _wander and _jiggleCount past this point.  It looks strange as
    // _lastOffset is set to 0 by rstclock().
  } else {
    // Compute the clock jitter as the RMS of exponentially weighted offset differences.
    // This is used by the poll-adjust code.
    etemp = sq(_clockJitter);
    dtemp = sq(max(fabs(offset - _lastOffset), LOG2D(S_PRECISION)));
    _clockJitter = sqrt(etemp + (dtemp - etemp) / AVG);
    switch (_clockState) {
      // In NSET state, this is the first update received and the frequency has not been
      // initialized.  The first thing to do is directly measure the oscillator frequency.
      case NSET:
        rstclock(FREQ, unixLFP, offset);
        return SLEW; // RFC returns IGNORE, but SLEW suits us better and follows SYNC->SPIK pattern.

      // In FSET state, this is the first update and the frequency has been initialized.
      // Adjust the phase, but don't adjust the frequency until the next update.
      case FSET:
        rstclock(SYNC, unixLFP, offset);
        break;

      // In FREQ state, ignore updates until the stepout threshold.
      // After that, correct the phase and frequency and switch to SYNC state.
      case FREQ:
        // XXX: RFC says `c.t - s.t < WATCH`, that translates to `$now - _lastUnix < WATCH`.
        // NTP-4.2.8p15 says `mu < WATCH` in the clock discipline comment.
        // Note, _clockJitter is always updated while offset might be ignored!
        if (mu < WATCH)
          return IGNORE;

        freq = (offset - clockOffset) / mu;
        rstclock(SYNC, unixLFP, offset);
        break;

      // We get here by default in SYNC and SPIK states.
      // Here we compute the frequency update due to PLL and FLL contributions.
      default:
        // The FLL and PLL frequency gain constants depending on the poll interval and Allan
        // intercept.  This code does not follow RFC5905, but rather follows reference
        // implementation of NTP-4.2.8p18 + bug 3053.  Initial frequency clamp (`freq_cnt`)
        // delaying frequency updates for stepout threshold ("WATCH" here, "CLOCK_MINSTEP" of 300
        // in NTP-4.2.8p18) is not used as I expect this code to be mostly used with ntppool
        // servers.  So, expected MINPOLL is 11 and WATCH of either 300 or 900 is irrelevant.
        const double CLOCK_FLL = 0.25;
        const double CLOCK_PLL = 16;
        if (_pollExponent >= ALLAN_XPT)
          freq += (offset - clockOffset) * CLOCK_FLL /
                  max(mu, LOG2D(_pollExponent));

        etemp = min(mu, LOG2D(ALLAN_XPT));
        dtemp = 4 * CLOCK_PLL * LOG2D(_pollExponent);
        freq += offset * etemp / (dtemp * dtemp);
        rstclock(SYNC, unixLFP, offset);
        break;
    }
  }

  // Calculate the new frequency and frequency stability (wander).  Compute the clock wander
  // as the RMS of exponentially weighted frequency differences.  This is not used directly,
  // but can, along with the jitter, be a highly useful monitoring and debugging tool.
  const double freqNext = freq + _freq;
  _freq = max(min(MAXFREQ, freqNext), -MAXFREQ);

#ifdef NTPCLIENT_WANDER
  // TODO: RFC code computes RMS of freqNext, however the RFC text suggests that RMS of "frequency
  // difference" might be calculated. Reference implementation calculates it as a clock_stability.
  etemp = sq(_wander);
  dtemp = sq(freq);
  _wander = sqrt(etemp + (dtemp - etemp) / AVG);
#endif

  // Here we adjust the poll interval by comparing the current offset with the clock jitter.
  // If the offset is less than the clock jitter times a constant, then the averaging interval
  // is increased; otherwise, it is decreased.  A bit of hysteresis helps calm the dance.
  // "The current offset" is _lastOffset as it was updated with rstclock() above.
  //
  // XXX: RFC 5905 and ntp-4.2.8p15 say >LIMIT whitepaper "Technical Report 06-6-1: NTPv4 Reference
  // and Implementation Guide" says >=LIMIT at https://www.ntp.org/reflib/reports/ntp4/ntp4.pdf
  //
  // NTP-4.2.8 adds time guards (tc_twinlo and tc_twinhi) for _pollExponent update, but those
  // are crucial only in case of multiple peers.  It works "as expected" for single peer with
  // the expectation being defined as "The 'tc_counter' dance itself is something that *should*
  // happen *once* every (1 << sys_poll) seconds" and local_clock() is never called more often.
  if (fabs(_lastOffset) < PGATE * _clockJitter) {
    _jiggleCount += _pollExponent;
    if (_jiggleCount > LIMIT) {
      _jiggleCount = LIMIT;
      if (_pollExponent < MAXPOLL) {
        _jiggleCount = 0;
        _pollExponent++;
      }
    }
  } else {
    _jiggleCount -= _pollExponent << 1;
    if (_jiggleCount < -LIMIT) {
      _jiggleCount = -LIMIT;
      if (_pollExponent > minpoll()) {
        _jiggleCount = 0;
        _pollExponent--;
      }
    }
  }
  return rval;
}

void NTPClient::step_time(double offset) {
  ASSUME(-PANICT <= offset && offset <= PANICT);
  // TODO: should _filter be also updated? Some NTP implementations do that.
  _lastMicros -= offset * 1e6;
}

void NTPClient::rstclock(unsigned char clockState, uint64_t unixLFP, double offset) {
  _clockState = clockState;

  step_time(offset);
  _lastOffset = offset;

  const uint32_t unix = unixLFP >> 32;
  const uint32_t since = unix - _lastUnix;
  _lastUnix += since;
  _lastUnixLFP16 = unixLFP >> 16;
  const int64_t sinceMicrosRaw = UINT64_C(1000000) * since;
  _lastMicros += sinceMicrosRaw - freqErr(sinceMicrosRaw);
}

#if 0
struct NTPClock {
  tstamp t;      // update time
  int state;     // current state
  double offset; // current offset
  double last;   // previous offset
  int count;     // jiggle counter
  double freq;   // frequency
  double jitter; // RMS jitter
  double wander; // RMS wander
} c;

struct s { // System structure
  tstamp  t;              /* update time */
  char    leap;           /* leap indicator */
  char    stratum;        /* stratum */
  char    poll;           /* poll interval */
  char    precision;      /* precision */
  double  rootdelay;      /* root delay */
  double  rootdisp;       /* root dispersion */
  char    refid;          /* reference ID */
  tstamp  reftime;        /* reference time */
  struct m m[NMAX];       /* chime list */
  struct v v[NMAX];       /* survivor list */
  struct p *p;            /* association ID */
  double  offset;         /* combined offset */
  double  jitter;         /* combined jitter */
  int     flags;          /* option flags */
  int     n;              /* number of survivors */
} s;

/* root_dist() - calculate root distance */
double
root_dist(struct p *p /* peer structure pointer */) {
  /*
   * The root synchronization distance is the maximum error due to all causes of the local clock
   * relative to the primary server.  It is defined as half the total delay plus total dispersion
   * plus peer jitter.
   */
  return (max(MINDISP, p->rootdelay + p->delay) / 2 +
      p->rootdisp + p->disp + PHI * (c.t - p->t) + p->jitter);
}

/*
 * fit() - test if association p is acceptable for synchronization
 */
int
fit( struct peer *p             /* peer structure pointer */)
{
  /* A stratum error occurs if (1) the server has never been synchronized, (2) the server stratum is
   * invalid.  */
  if (p->leap == NOSYNC || p->stratum >= MAXSTRAT)
    return (FALSE);

  /*
   * A distance error occurs if the root distance exceeds the distance threshold plus an increment
   * equal to one poll interval.
   */
  if (root_dist(p) > MAXDIST + PHI * LOG2D(s.poll))
    return (FALSE);

  /*
   * A loop error occurs if the remote peer is synchronized to the local peer or the remote peer is
   * synchronized to the current system peer.  Note this is the behavior for IPv4; for IPv6 the MD5
   * hash is used instead.
   */
  if (p->refid == p->dstaddr || p->refid == s.refid)
    return (FALSE);

  /* An unreachable error occurs if the server is unreachable.
   */
  if (p->reach == 0)
    return (FALSE);

  return (TRUE);
}


// rstclock() - clock state machine
void rstclock(int state,     /* new state */
              double offset, /* new offset */
              double t       /* new update time */
) {
  // Enter new state and set state variables.  Note, we use the time of the last clock filter
  // sample, which must be earlier than the current time.
  c.state = state;
  c.last = c.offset = offset;
  s.t = t;
}


// clock_update() - update the system clock
void clock_update(struct p* p /* peer structure pointer */) {
  double dtemp;

  // If this is an old update, for instance, as the result of a system peer change, avoid it.
  // We never use an old sample or the same sample twice.
  if (s.t >= p->t)
    return;

  // Combine the survivor offsets and update the system clock; the local_clock() routine will tell
  // us the good or bad news.
  s.t = p->t;
  clock_combine();
  switch (local_clock(p, s.offset)) {
    // The offset is too large and probably bogus.  Complain to the system log and order the
    // operator to set the clock manually within PANIC range.  The reference implementation includes
    // a command line option to disable this check and to change the panic threshold from the
    // default 1000 s as required.
    case PANIC:
      exit(0);

    // The offset is more than the step threshold (0.125 s by default).  After a step, all
    // associations now have inconsistent time values, so they are reset and started fresh.
    // The step threshold can be changed in the reference implementation in order to lessen
    // the chance the clock might be stepped backwards.  However, there may be serious
    // consequences, as noted in the white papers at the NTP project site.
    case STEP:
      while (/* all associations */ 0)
        clear(p, X_STEP);
      s.stratum = MAXSTRAT;
      s.poll = MINPOLL;
      break;

    // The offset was less than the step threshold, which is the normal case.  Update the system
    // variables from the peer variables.  The lower clamp on the dispersion increase is to avoid
    // timing loops and clockhopping when highly precise sources are in play.  The clamp can be
    // changed from the default .01 s in the reference implementation.
    case SLEW:
      s.leap = p->leap;
      s.stratum = p->stratum + 1;
      s.refid = p->refid;
      s.reftime = p->reftime;
      s.rootdelay = p->rootdelay + p->delay;
      dtemp = sqrt(sq(p->jitter) + sq(s.jitter));
      dtemp += max(p->disp + PHI * (c.t - p->t) + fabs(p->offset), MINDISP);
      s.rootdisp = p->rootdisp + dtemp;
      break;

    // Some samples are discarded while, for instance, a direct frequency measurement is being made.
    case IGNORE:
      break;
  }
}
#endif

unsigned long NTPClient::nextPollInterval() const {
  // 1024ms is not exactly 1s, but that's close enough given added jitter
  // Note, Popcorn Spike Suppressor MIGHT assume that that pollIn is an upper bound.
  // NTP-4.2.8p18 schedules next poll in [1...17/16) interval and I trust Mills' judgement.
  unsigned long pollIn = 1UL << (_pollExponent + 10);
  unsigned long sixteenth = 1UL << (_pollExponent + 6);
  unsigned long jitter = random() & (sixteenth - 1);
  return pollIn + jitter;  // pollIn * [1...17/16)
}

void NTPClient::scheduleNextPoll() {
  _nextPollMillis = millis() + nextPollInterval();
  _waitingForReply = false;
}

bool NTPClient::update() {
  bool last;
  do {
    last = maintain();
  } while (_waitingForReply);
  return last;
}

bool NTPClient::isTimeSet() const {
  return _clockState != NCLK;
}

bool NTPClient::isTimeSync() const {
  return _clockState == SYNC;
}

bool NTPClient::getTimeOfDay(NTPTimeval *tv) const {
  if (!isTimeSet())
    return false;
  const uint64_t sinceMicros = micros63() - _lastMicros;
  tv->tv_usec = sinceMicros % 1000000;
  const uint64_t sinceSec = (sinceMicros - tv->tv_usec) / 1000000;
  tv->tv_sec = _lastUnix + sinceSec;
  return true;
}

int64_t NTPClient::freqErr(int64_t dtMicros) const {
  if (_clockState < FSET)
    return 0;
  const int64_t err = dtMicros * _freq;
  return err;
}

uint64_t NTPClient::unixLFPAt(uint64_t micros) const {
  // ASSUME(_lastMicros <= micros);
  const int64_t sinceMicrosRaw = micros - _lastMicros;
  const int64_t sinceMicros = sinceMicrosRaw + freqErr(sinceMicrosRaw);
  const uint32_t sinceSec = sinceMicros / 1000000;
  const uint32_t sinceLfplo = us2lfplo(sinceMicros % 1000000);
  const uint32_t sinceLfphi = _lastUnix + sinceSec;
  const uint64_t since = (uint64_t(sinceLfphi) << 32) | sinceLfplo;
  return since;
}

unsigned long NTPClient::getEpochTime() const {
  const int64_t sinceMicrosRaw = micros63() - _lastMicros;
  const int64_t sinceMicros = sinceMicrosRaw + freqErr(sinceMicrosRaw);
  const uint32_t sinceSec = sinceMicros / 1000000;
  return _timeOffset + _lastUnix + sinceSec;
}

int NTPClient::getDay() const {
  return (((this->getEpochTime()  / 86400L) + 4 ) % 7); //0 is Sunday
}
int NTPClient::getHours() const {
  return ((this->getEpochTime()  % 86400L) / 3600);
}
int NTPClient::getMinutes() const {
  return ((this->getEpochTime() % 3600) / 60);
}
int NTPClient::getSeconds() const {
  return (this->getEpochTime() % 60);
}

String NTPClient::getFormattedTime() const {
  unsigned long rawTime = this->getEpochTime();
  unsigned long hours = (rawTime % 86400L) / 3600;
  String hoursStr = hours < 10 ? "0" + String(hours) : String(hours);

  unsigned long minutes = (rawTime % 3600) / 60;
  String minuteStr = minutes < 10 ? "0" + String(minutes) : String(minutes);

  unsigned long seconds = rawTime % 60;
  String secondStr = seconds < 10 ? "0" + String(seconds) : String(seconds);

  return hoursStr + ":" + minuteStr + ":" + secondStr;
}

void NTPClient::end() {
  if (this->_udpSetup)
    this->_udp.stop();
  this->_udpSetup = false;
}

void NTPClient::setTimeOffset(int timeOffset) {
  this->_timeOffset     = timeOffset;
}

void NTPClient::setUpdateInterval(unsigned long updateInterval) {
  this->_pollExponent   = millisToPollExponent(updateInterval);
  if (isPollingNtppool())
    this->_pollExponent = max(_pollExponent, (uint8_t)NTPPOOL_MINPOLL);
}

void NTPClient::setPoolServerName(const char* poolServerName) {
    this->_poolServerName = poolServerName;
    this->_doResolve = true;
    if (isPollingNtppool())
      this->_pollExponent = max(_pollExponent, (uint8_t)NTPPOOL_MINPOLL);
}

void NTPClient::sendNTPPacket(const unsigned char xmt[8]) {
  // TODO: sendNTPPacket() deserves some error checking
  if  (_doResolve) {
    this->_udp.beginPacket(this->_poolServerName, PORT);
  } else {
    this->_udp.beginPacket(this->_poolServerIP, PORT);
  }
  // Values are picked per NTP Client Data Minimization draft:
  // https://datatracker.ietf.org/doc/html/draft-ietf-ntp-data-minimization-04
  uint8_t pkt[NTP_PACKET_SIZE];
  pkt[0] = 0x23;  // (LI = 0, VN = 4, Mode = 3).
  pkt[1] = 0;     // Stratum = 0
  pkt[2] = _pollExponent;
  pkt[3] = 0x20;  // Clock Precision = 136 years
  memset(pkt + 4, 0, NTP_PACKET_SIZE - 4 - 8);
  memcpy(pkt + XMT_OFFSET, xmt, 8);  // Transmit Timestamp field SHOULD be random
  this->_udp.write(pkt, NTP_PACKET_SIZE);
  _t1 = micros();
  this->_udp.endPacket();
}

void NTPClient::setRandomPort(unsigned int minValue, unsigned int maxValue) {
  this->_port = random(minValue, maxValue + 1);
  this->_keepPortOnNewIP = true;
  end();
}
