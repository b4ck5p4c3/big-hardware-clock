#pragma once

#include "Arduino.h"

#include <Udp.h>

#ifndef ARDUINO_NTP_SERVER
// The NTP pool is a shared resource provided by the NTP Pool Project[POOL] and used by people
// and services all over the world. To prevent it from becoming overloaded, please avoid querying
// the standard `pool.ntp.org` zone names in your applications. Instead, consider requesting your
// own vendor zone[ZONE] and/or joining the pool[POOL].
//
// [POOL] https://www.ntppool.org/en/
// [ZONE] https://www.ntppool.org/en/vendors.html
// [POOL] https://www.ntppool.org/join.html
#warning Look at www.ntppool.org/en/vendors.html before using "pool.ntp.org".
#define ARDUINO_NTP_SERVER "pool.ntp.org"
#endif

#ifndef ARDUINO_NTP_MINPOLL
// NTP Pool Terms of Service
// 4. End-User agrees that he or she will not:
// (b) Request time from the Services more than once every thirty (30) minutes [...], if using SNTP.
// -- https://www.ntppool.org/tos.html
#define ARDUINO_NTP_MINPOLL 11 // 34m8s
#endif

#ifndef ARDUINO_NTP_REJECTD_CACHE
#define ARDUINO_NTP_REJECTD_CACHE 4 // number of `A` replies for pool.ntp.org
#endif

#define DEFPOLL 6 // 64s for compatibility with NTPClient-3.2.x

class NTPClock {
  private:
    unsigned long _refUnix;   // unsigned 32-bit time_t is okay till 2106
    unsigned long _refMillis; // adjusted to align with the start of _refUnix second
  public:
    NTPClock() : _refUnix(0), _refMillis(0) {}
    bool isSet(void) const { return _refUnix != 0; }
    void set(unsigned long unix, unsigned long millis) {
      _refUnix = unix;
      _refMillis = millis;
    }
    int offset(unsigned long unix, unsigned long millis) const {
      // (dtSec * 1000UL) overflow (49 days) still produces correct offset as millis() wrap
      const unsigned long dtSec = unix - _refUnix;
      const unsigned long localMillis = _refMillis + (dtSec * 1000UL);
      const long offsetLong = (long)(millis - localMillis);
      return offsetLong > INT_MAX   ? INT_MAX
             : offsetLong < INT_MIN ? INT_MIN
                                    : offsetLong;
    }
    unsigned long getEpochTime(unsigned long *pMillis) const {
      const unsigned long now = millis();
      const unsigned long dtMillis = now - _refMillis;
      const unsigned long dtSec = dtMillis / 1000;
      if (pMillis)
        *pMillis = dtMillis % 1000;
      return _refUnix + dtSec;
    }
};

class NTPClient {
  private:
    UDP*          _udp;
    const char*   _serverName;
    IPAddress     _serverIP; // cached till KoD or timeout
    NTPClock      _clock;
    double        _clockJitter;
    unsigned long _lastPollMillis;
    unsigned char _pollExponent;
    unsigned char _reach;    // 8-bit integer shift register
    unsigned char _unreach;  // unreach counter
    signed char   _jiggle;   // jiggle counter
    int           _lastOffsetMillis; // [-32s; +32s]

    unsigned short _srcPort;
    unsigned char  _flags;
    static const unsigned char _confSrcPort = 1U << 0; // user set port explicitly
    static const unsigned char _udpSetup = 1U << 1;
    static const unsigned char _tryDNS = 1U << 2;
    static const unsigned char _firstPollDone = 1U << 3;

    // As of 2025, there are UTC offsets ranging from -12h00m to +14h00m with granularity of 15m.
    // Other granularities are legacy, so storing offset as a number of 15m intervals is okay while
    // saving some memory.
    signed char   _QHourOffset;

    unsigned char ntppoolExponent(unsigned char pollExponent) const;
    void          resetNtpPoll(unsigned char pollExponent);
    unsigned long sendNTPPacket(const unsigned char xmt[8]);
    void          unreachPollInterval(void);
    void          jigglePollInterval(int offsetMillis);
    void          ingestGoodTime(unsigned char *pkt, unsigned long t1, unsigned long t4);
    bool          isTimeToPoll(void) const;

  public:
    static const unsigned long poolInterval = 1000UL * (1UL << ARDUINO_NTP_MINPOLL);
    static const unsigned long nonpoolInterval = 1000UL * (1UL << DEFPOLL);

    NTPClient(UDP &udp, long timeOffset = 0);
    NTPClient(UDP &udp, const char *serverName, long timeOffset = 0,
              unsigned long updateInterval = nonpoolInterval);
    NTPClient(UDP &udp, IPAddress serverIP, long timeOffset = 0,
              unsigned long updateInterval = nonpoolInterval);

    /**
     * Set time server name
     *
     * @param serverName
     */
    void setPoolServerName(const char* serverName);

     /**
     * Set random local port
     */
    void setRandomPort(unsigned int minValue = 49152, unsigned int maxValue = 65535);

    /**
     * Starts the underlying UDP client with the specified local port. Port 0 uses default value.
     *
     * @return 1 if successful, 0 if there are no sockets available to use
     */
    int begin(unsigned int port = 0);

    /**
     * This should be called in the main loop of your application. By default an update from the NTP Server is only
     * made every 60 seconds. This can be configured in the NTPClient constructor.
     *
     * @return true on success, false on failure
     */
    bool update();

    /**
     * This will force the update from the NTP Server.
     *
     * @return true on success, false on failure
     */
    bool forceUpdate();

    /**
     * This allows to check if the NTPClient successfully received a NTP packet and set the time.
     *
     * @return true if time has been set, else false
     */
    bool isTimeSet() const;

    int getDay() const;
    int getHours() const;
    int getMinutes() const;
    int getSeconds() const;

    /**
     * Changes the time offset. Useful for changing timezones dynamically
     */
    void setTimeOffset(int timeOffset);

    /**
     * Set the update interval to another frequency. E.g. useful when the
     * timeOffset should not be set in the constructor
     */
    void setUpdateInterval(unsigned long updateInterval);

    /**
     * @return time formatted like `hh:mm:ss`
     */
    String getFormattedTime() const;

    /**
     * @return time in seconds since Jan. 1, 1970
     */
    unsigned long getEpochTime(unsigned long *pMillis = 0) const;

    /**
     * Stops the underlying UDP client
     */
    void end();
};
