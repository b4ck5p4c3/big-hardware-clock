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

#define PORT 123
#define ROOTDELAY_OFFSET 4
#define ROOTDISP_OFFSET 8
#define REF_OFFSET 16
#define ORG_OFFSET 24
#define REC_OFFSET 32
#define XMT_OFFSET 40
#define NTP_PACKET_SIZE 48
#define NTP_TIMEOUT_MS 1000

#define JAN_1970 2208988800UL // 1970 - 1900 in seconds

// UNREACH value from RFC5905 is 12, but https://www.ntp.org/documentation/4.2.8-series/poll/
// defines it as 10, so this code follows that guidance. PGATE and LIMIT are the same.
#define AVG 4          // parameter averaging constant
#define PGATE 4        // poll-adjust gate for jitter/offset ratio
#define LIMIT 30       // poll-adjust threshold for jiggle counter
#define UNREACH 10     // unreach counter threshold
#define KODSTRAT 0     // Kiss-o'-Death stratum number
#define MAXSTRAT 16    // maximum stratum number
#define MAXDIST 1      // 1s, root distance threshold
#define MAXDISP 16     // 16s
#define MAXPOLL 17     // 36h, maximum poll exponent
#define MINPOLL_NTP 4  // 16s, minimum poll exponent for NTPv4, RFC5905
#define MINPOLL_SNTP 6 // 64s, minimum poll exponent for SNTP, RFC4330 Section 5

#define MINPOLL MINPOLL_SNTP // That's not NTP, right?
#define MAXDISP_MS (MAXDISP * 1000)

#define ARRAY_LEN(x) (sizeof(x) / sizeof((x)[0]))
#define ARRAY_END(x) (x + ARRAY_LEN(x))
#define IS_POW2(x) (((x) & (x - 1)) == 0)

static const uint32_t INADDR_ANY = 0;

static signed char secondsToQHour(long timeOffset) { return timeOffset / 900; }

static int clzul(unsigned long x) {
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

static unsigned char millisecondsToPollExponent(unsigned long updateInterval) {
  updateInterval /= 1000;
  unsigned char pollExponent = log2ul(updateInterval);
  if (!IS_POW2(updateInterval))
    pollExponent++; // 3600_000ms should map to 64s, not 32s ;-)
  if (pollExponent < MINPOLL)
    pollExponent = MINPOLL;
  if (pollExponent > MAXPOLL)
    pollExponent = MAXPOLL;
  return pollExponent;
}

static void randomSeed() {
  unsigned long r = random(LONG_MAX); // in case if it's already seeded
  for (int i = 0; i < sizeof(r) * 8; i++)
    r ^= ((unsigned long)analogRead(0)) << i; // analogRead() is just 10..12 bits
  randomSeed(r);
}

static bool isBadRootDistance(unsigned char *pkt) {
  const unsigned long rootdelay =
      pkt[ROOTDELAY_OFFSET + 0] << 24 | pkt[ROOTDELAY_OFFSET + 1] << 16 |
      pkt[ROOTDELAY_OFFSET + 2] << 8 | pkt[ROOTDELAY_OFFSET + 3];
  const unsigned long rootdisp =
      pkt[ROOTDISP_OFFSET + 0] << 24 | pkt[ROOTDISP_OFFSET + 1] << 16 |
      pkt[ROOTDISP_OFFSET + 2] << 8 | pkt[ROOTDISP_OFFSET + 3];
  // That's (rootdelay / 2 + rootdisp >= MAXDIST), adjusted for possible overflow.
  const unsigned long maxdist = MAXDIST << 16;
  return (rootdelay >> 2) + (rootdisp >> 1) >= (maxdist >> 1);
}

NTPClient::NTPClient(UDP& udp, long timeOffset) {
  randomSeed();
  _udp = &udp;
  _flags = 0;
  _srcPort = 0;
  setTimeOffset(timeOffset);
  _serverName = ARDUINO_NTP_SERVER;
  _serverIP = INADDR_ANY;
  resetNtpPoll(ARDUINO_NTP_MINPOLL);
}

NTPClient::NTPClient(UDP& udp, const char* serverName, long timeOffset, unsigned long updateInterval) {
  randomSeed();
  _udp = &udp;
  _flags = 0;
  _srcPort = 0;
  setTimeOffset(timeOffset);
  _serverName = serverName;
  _serverIP = INADDR_ANY;
  setUpdateInterval(updateInterval);
}

NTPClient::NTPClient(UDP& udp, IPAddress poolServerIP, long timeOffset, unsigned long updateInterval) {
  randomSeed();
  _udp = &udp;
  _flags = 0;
  _srcPort = 0;
  setTimeOffset(timeOffset);
  _serverName = NULL;
  _serverIP = poolServerIP;
  setUpdateInterval(updateInterval);
}

unsigned char NTPClient::ntppoolExponent(unsigned char pollExponent) const {
  if (_serverName && strcmp(_serverName, ARDUINO_NTP_SERVER) == 0)
    if (pollExponent < ARDUINO_NTP_MINPOLL)
      return ARDUINO_NTP_MINPOLL;
  return pollExponent;
}

void NTPClient::resetNtpPoll(unsigned char pollExponent) {
  _pollExponent = pollExponent;
  _reach = _unreach = _jiggle = 0;
  // _srcPort is reset as soon as associated is reset, new association gets new port per RFC 9109.
  if (!(_flags & _confSrcPort))
    _srcPort = 0;
  if (_flags & _udpSetup)
    this->end();
}

void NTPClient::unreachPollInterval(void) {
  if (_reach) {
    _unreach = 0; // The server is reachable.
  } else if (_unreach < UNREACH) {
    // The server is unreachable, so bump the unreach counter.  If the unreach threshold has been
    // reached, double the poll interval to minimize wasted network traffic.
    _unreach++;
  } else {
    // If the timer exceeds the unreach threshold set at 10, the poll exponent is incremented by 1
    // and the unreach timer set to 0. -- https://www.ntp.org/documentation/4.2.8-series/poll/
    _pollExponent = min(_pollExponent + 1, MAXPOLL);
    _unreach = 0;
  }
}

void NTPClient::jigglePollInterval(int offsetMillis) {
  // Compute the clock jitter as the RMS of exponentially weighted offset differences.
  // This is used by the poll-adjust code.
  const int dOffset = offsetMillis - _lastOffsetMillis;
  const double etemp = sq(_clockJitter);
  const double dtemp = sq(max(fabs(dOffset * 0.001), 0.001));
  _clockJitter = sqrt(etemp + (dtemp - etemp) / AVG);

  // Here we adjust the poll interval by comparing the current offset with the clock jitter.
  // If the offset is less than the clock jitter times a constant, then the averaging interval
  // is increased; otherwise, it is decreased.  A bit of hysteresis helps calm the dance.
  if (fabs(offsetMillis * 0.001) < PGATE * _clockJitter) {
    _jiggle += _pollExponent;
    if (_jiggle > LIMIT) {
      _jiggle = LIMIT;
      if (_pollExponent < MAXPOLL) {
        _jiggle = 0;
        _pollExponent++;
      }
    }
  } else {
    _jiggle -= _pollExponent << 1;
    if (_jiggle < -LIMIT) {
      _jiggle = -LIMIT;
      if (_pollExponent > MINPOLL) {
        _jiggle = 0;
        _pollExponent--;
      }
    }
  }
}

int NTPClient::begin(unsigned int port) {
  // Port is randomized per RFC 6056 (Recommendations for Transport-Protocol Port Randomization).
  if (port == 0)
    port = (_flags & _confSrcPort) ? _srcPort : random(1024, 65535);
  else
    _flags |= _confSrcPort;
  // It's persisted to reduce Effects on Path Selection per RFC 9109 (NTPv4: Port Randomization).
  if (_srcPort == 0)
    _srcPort = port;
  const int ret = _udp->begin(port);
  if (ret == 1)
    _flags |= _udpSetup;
  return ret;
}

bool NTPClient::forceUpdate() {
  if (!(_flags & _udpSetup))
    if (this->begin() != 1)
      return false;

  #ifdef DEBUG_NTPClient
    Serial.println("Update from NTP Server");
  #endif

  // flush any existing packets
  while(this->_udp->parsePacket() != 0)
    this->_udp->flush();

  // Shift the reachability register to the left.  Hopefully, some time before the next poll
  // a packet will arrive and set the rightmost bit.
  _reach <<= 1;
  unreachPollInterval();

  unsigned short randTx[4];
  for (unsigned short *p = randTx; p != ARRAY_END(randTx); p++) {
    const long r = random();
    *p = (unsigned short)(r ^ (r >> 16));
  }
  static_assert(sizeof(randTx) == 8);
  const unsigned long t1 = sendNTPPacket((unsigned char*)randTx);

  unsigned long t4 = t1;
  do {
    delay(10);
    const int sz = _udp->parsePacket();
    t4 = millis();
    if (sz != NTP_PACKET_SIZE || _udp->remotePort() != PORT)
      continue; // not a relevant NTP packet for sure

    const unsigned char verAndMode = 0b00111111;
    const unsigned char ver4Server = 0b00100100; // NTPv4, Mode=4 (server)
    unsigned char pkt[NTP_PACKET_SIZE];
    _udp->read(pkt, NTP_PACKET_SIZE);
    // FIXME: using `memcmp() > 0` does wrong thing durring NTP rollover in 2036.
    if (memcmp(pkt + ORG_OFFSET, randTx, 8) != 0 ||
        (pkt[0] & verAndMode) != ver4Server || pkt[1] > MAXSTRAT ||
        memcmp(pkt + REC_OFFSET, pkt + XMT_OFFSET, 8) > 0 ||
        memcmp(pkt + REF_OFFSET, pkt + XMT_OFFSET, 8) > 0)
      continue; // mismatching cookie and/or malformed packet

    const IPAddress remoteIP = _udp->remoteIP();
    if (INADDR_ANY == _serverIP) {
      _serverIP = remoteIP;
    } else if (_serverIP == remoteIP) {
      ; // no-op
    } else if (_flags & _tryDNS) {
      // _pollExponent is kept intact and not reset to updateInterval as _tryDNS might be trigged
      // by KODSTRAT.  Also, jigglePollInterval() should work if offset differs measurably.
      resetNtpPoll(_pollExponent);
      _serverIP = remoteIP;
    } else if (!(_flags & _tryDNS)) {
      continue; // reply from bogus remoteIP
    }
    _flags &= ~_tryDNS;

    // Now, we're confident that we got The Reply from The Server.

    if (pkt[1] == KODSTRAT) {
      // An SNTP client should stop sending to a particular server if that server returns a reply
      // with a Stratum field of 0 (kiss-o'-death packet), regardless of kiss code, and an alternate
      // server is available.  If no alternate server is available, the client should retransmit
      // using an exponential backoff algorithm. -- RFC 4330: SNTPv4
      //
      // Unfortunately, UDP class does not provide API for DNS, so we don't really know if we have
      // an alternate server or not.  So, we have to resort to doing next query with DNS resolution
      // and increasing poll interval.  New IP triggers resetNtpPoll().
      _pollExponent = min(_pollExponent + 1, MAXPOLL);
      _flags |= _tryDNS;
      break;
    }

    // Verify the server is synchronized with valid stratum and reference time not later
    // than the transmit time.  Verify valid root distance.
    const unsigned char leapNosync = 0b11000000;
    if ((pkt[0] & leapNosync) != leapNosync && pkt[1] != MAXSTRAT && !isBadRootDistance(pkt)) {
      _reach |= 1;
      ingestGoodTime(pkt, t1, t4);
    } else {
      _flags |= _tryDNS;
    }
    break;
  } while (t4 - t1 < NTP_TIMEOUT_MS);

  if (!_reach)
    _flags |= _tryDNS;
  _lastPollMillis = t4;
  _flags |= _firstPollDone;

  return (_reach & 1) ? true : false;
}

void NTPClient::ingestGoodTime(unsigned char *pkt, unsigned long t1millis, unsigned long t4millis) {
  // That's basic SNTP math done in fixed point arithmetics to avoid FPU usage on MCU. *sec parts
  // are full NTP seconds, *p32 parts are 1/2^32 parts of second, fractional parts of NTP seconds,
  // *millis are milliseconds.  (A+B)/2 is expressed as (A+(B-A)/2) to deal with wrapping overflow.
  const unsigned long t2sec = pkt[REC_OFFSET + 0] << 24 | pkt[REC_OFFSET + 1] << 16 |
                              pkt[REC_OFFSET + 2] << 8 | pkt[REC_OFFSET + 3];
  const unsigned long t2p32 = pkt[REC_OFFSET + 4] << 24 | pkt[REC_OFFSET + 5] << 16 |
                              pkt[REC_OFFSET + 6] << 8 | pkt[REC_OFFSET + 7];
  const unsigned long t3sec = pkt[XMT_OFFSET + 0] << 24 | pkt[XMT_OFFSET + 1] << 16 |
                              pkt[XMT_OFFSET + 2] << 8 | pkt[XMT_OFFSET + 3];
  const unsigned long t3p32 = pkt[XMT_OFFSET + 4] << 24 | pkt[XMT_OFFSET + 5] << 16 |
                              pkt[XMT_OFFSET + 6] << 8 | pkt[XMT_OFFSET + 7];
  // delay = (T4 - T1) - (T3 - T2)
  const unsigned long dt32sec = t3sec - t2sec - ((t3p32 < t2p32) ? 1 : 0);
  const unsigned long dt32p32 = t3p32 - t2p32;
  const unsigned long dt32millis = dt32sec * 1000 + dt32p32 / 4294967UL; // 2^32 / 1000 ≈ 4294967.296
  const unsigned long dt41millis = t4millis - t1millis;
  const unsigned long delayLong = dt41millis - dt32millis;
  const unsigned int delayMillis = delayLong > MAXDISP_MS ? MAXDISP_MS
                                   : delayLong == 0       ? 1
                                                          : delayLong;
  // offset = ((T2 + T3) - (T1 + T4)) / 2
  const unsigned msb = sizeof(unsigned long) * 8 - 1;
  const unsigned long midp23p32 = t2p32 + ((dt32sec & 1) << msb) + dt32p32 / 2;
  // At most one carry might happen above, e.g. 0xFF + 0x80 + 0x7F -> 0x1FE
  const unsigned long midp23sec = t2sec + dt32sec / 2 + (midp23p32 < t2p32 ? 1 : 0);

  const unsigned long midp23Unix = midp23sec - JAN_1970;
  const unsigned long mipd14millis = t1millis + dt41millis / 2 - (midp23p32 / 4294967UL);

  if (_clock.isSet()) {
    const int offsetMillis = _clock.offset(midp23Unix, mipd14millis);
    jigglePollInterval(offsetMillis); // When a clock update is received...
    _lastOffsetMillis = offsetMillis;
  }
  // SNTP STEPs on every sample, has no clock discipline and/or frequency adjustments.
  _clock.set(midp23Unix, mipd14millis);
}

bool NTPClient::isTimeToPoll(void) const {
  // dtMillis overflows and wraps in ≈49 days, MAXPOLL is 36h, so it should be okay.
  const unsigned long now = millis();
  const unsigned long dtMillis = now - _lastPollMillis;
  const unsigned long pollSeconds = 1UL << _pollExponent;
  if (dtMillis > (pollSeconds << 10)) // rough estimate avoiding MUL
    return true;
  const unsigned long updateInterval = pollSeconds * 1000UL; // precise one
  return dtMillis > updateInterval;
}

bool NTPClient::update() {
  if (!(_flags & _firstPollDone) || isTimeToPoll())
    return forceUpdate();
  return false;
}

bool NTPClient::isTimeSet() const {
  return _clock.isSet();
}

unsigned long NTPClient::getEpochTime(unsigned long *pMillis) const {
  return _QHourOffset * 900 + _clock.getEpochTime(pMillis);
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
  _udp->stop();
  _flags &= ~_udpSetup;
  if (!(_flags & _confSrcPort))
    _srcPort = 0;
}

void NTPClient::setTimeOffset(int timeOffset) {
  _QHourOffset = secondsToQHour(timeOffset);
}

void NTPClient::setUpdateInterval(unsigned long updateInterval) {
  _pollExponent = ntppoolExponent(millisecondsToPollExponent(updateInterval));
}

void NTPClient::setPoolServerName(const char* serverName) {
  _serverName = serverName;
  _serverIP = INADDR_ANY;
  // TODO: keep `updateInterval` if it was set in ctor
  resetNtpPoll(ntppoolExponent(DEFPOLL));
}

unsigned long NTPClient::sendNTPPacket(const unsigned char xmt[8]) {
  // Values are set per https://datatracker.ietf.org/doc/html/draft-ietf-ntp-data-minimization-04
  unsigned char pkt[NTP_PACKET_SIZE];
  pkt[0] = 0x23;          // (LI = 0, VN = 4, Mode = 3).
  pkt[1] = 0;             // Stratum = 0
  pkt[2] = _pollExponent; // Polling Interval
  pkt[3] = 0x20;          // Peer Clock Precision = 136 years
  // XXX: Should I save 36 bytes of stack and call _udp->write(0) byte-by-byte?
  memset(pkt + 4, 0, NTP_PACKET_SIZE - 4 - 8);
  memcpy(pkt + 40, xmt, 8); // Transmit Timestamp field SHOULD be random

  // Use cached _serverIP value if it's a valid one, however, the cached value is updated later,
  // as _udp->remoteIP() "must be called after WiFiUDP.parsePacket()".
  if (INADDR_ANY == _serverIP || _serverName && (_flags & _tryDNS))
    _udp->beginPacket(_serverName, PORT);
  else
    _udp->beginPacket(_serverIP, PORT);

  const unsigned long t1 = millis();
  _udp->write(pkt, NTP_PACKET_SIZE);
  _udp->endPacket();
  return t1;
}

void NTPClient::setRandomPort(unsigned int minValue, unsigned int maxValue) {
  _flags |= _confSrcPort;
  _srcPort = random(minValue, maxValue);
}
