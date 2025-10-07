#pragma once

#include <Arduino.h>
#include <Udp.h>

/* That's SNTPv4 client supporting the following features:
 *  - NTP Client Data Minimization per draft-ietf-ntp-data-minimization-04
 *  - Port randomization per RFC 6056 and RFC 9109
 *  - MINPOLL of 11 (30 minutes) for ntppool per https://www.ntppool.org/tos.html
 *  - Server IP caching to reduce jitter and measure true NTP Delay (T4-T1) without DNS query RTT
 *  - Incoming packet validation (NTPClient-3.2.1 did not validate input)
 *  - Non-blocking network operation without delay() calls
 *  - Sub-second precision to make Arduino-based clocks "tick" synchronously
 */

// Some platforms (e.g. ESP32) have sys/time.h and struct timeval, and some don't (e.g. ATmega328).
#ifdef HAVE_STRUCT_TIMEVAL
struct timeval;
typedef struct timeval NTPTimeval;
#else
struct NTPTimeval {
  unsigned long tv_sec;
  unsigned long tv_usec;
};
#endif

class NTPClient {
  private:
    static const unsigned NSTAGE = 8; // clock register stages

    UDP&          _udp;

    const char*   _poolServerName = "pool.ntp.org"; // Default time server
    IPAddress     _poolServerIP;
    long          _timeOffset     = 0;

    // _lastMicros corrsespong to _lastUnix.00000000, not to _lastUnix._lastUnixLFP16
    // _lastUnixLFP16 is used for extra precision in `mu` calculation.
    int64_t       _lastMicros;
    uint32_t      _lastUnix;
    uint16_t      _lastUnixLFP16;
    double        _lastOffset;
    double        _clockJitter;
    double        _freq;
#ifdef NTPCLIENT_SYSLOG
    double        _wander;
#endif
    int8_t        _jiggleCount;
    uint8_t       _reach;

    struct Sample {
      uint32_t unixHigh;       // It does not matter much if it's NTP or Unix. That's unix.
      uint16_t unixLFP16;      // 16 high bits of 32-bit LFP. That's fine given S_PRECISION.
      uint16_t delayMicros16;  // This machine speaks micros(), that's count of 32us steps.
      double   offset;         // Offsets beyond ±125ms (STEPT) are either IGNORE'd or lead to STEP.
    };
    Sample        _filter[NSTAGE];

    union {
      uint16_t      _xmt16[4];        // not T1, but a random cookie
      unsigned long _nextPollMillis;  // invalid while _waitingForReply
    };
    unsigned long _t1;
    uint16_t      _port;
    unsigned char _pollExponent;
    bool          _keepPortOnNewIP : 1;
    bool          _udpSetup        : 1;
    bool          _doResolve       : 1;
    bool          _waitingForReply : 1;
    unsigned char _clockState      : 3;
    unsigned char _filterNextIndex : 3;

    unsigned long nextPollInterval() const;
    void          scheduleNextPoll();
    void          ctor(long timeOffset);
    void          clearAssociation();
    bool          sendRequest();
    bool          checkForReply();
    void          sendNTPPacket(const unsigned char xmt[8]);
    bool          isPollingNtppool(void) const;
    unsigned char minpoll(void) const;
    const Sample* bestSample(void) const;
    double        peerJitter(const Sample* best) const;
    uint64_t      unixLFPAt(uint64_t micros) const;
    int64_t       freqErr(int64_t micros) const;
    bool          clockFilter(uint64_t unix, double offset, uint32_t delayMicros);
    bool          clockUpdate(uint64_t unixLFP, double offset);
    unsigned char localClock(uint64_t unixLFP, double offset);
    void          step_time(double offset);
    void          rstclock(unsigned char clockState, uint64_t unixHigh, double offset);

  public:
    NTPClient(UDP& udp, long timeOffset = 0);
    NTPClient(UDP& udp, const char* poolServerName, long timeOffset = 0, unsigned long updateInterval = 60000);
    NTPClient(UDP& udp, IPAddress poolServerIP, long timeOffset = 0, unsigned long updateInterval = 60000);

    /**
     * Set time server name
     *
     * @param poolServerName
     */
    void setPoolServerName(const char* poolServerName);

     /**
     * Set random local port
     */
    void setRandomPort(unsigned int minValue = 1024, unsigned int maxValue = 65535);

    /**
     * Starts the underlying UDP client with the default local port
     */
    void begin();

    /**
     * Starts the underlying UDP client with the specified local port
     */
    void begin(unsigned int port);

    /**
     * This should be called on every main loop iteration of your application
     * to keep NTP-based clock up-to-date while avoiding delay() in loop().
     *
     * @return true when clock got update via NTP, false otherwise
     */
    bool maintain();

    /**
     * This might be called in the main loop of your application.  It checks if it's time to query
     * NTP server, polls it as needed and updates clock.  It blocks the loop().
     *
     * @return true on success, false on failure
     */
    bool update();

    /**
     * This will force the update from the NTP Server.  It blocks the loop().
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

    bool isTimeSync() const;
    uint8_t getPollExponent(void) const {
      return _pollExponent;
    }
    double getClockJitter(void) const {
      return _clockJitter;
    }
    double getClockFreq(void) const {
      return _freq;
    }
    double getClockWander(void) const {
      return _wander;
    }

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
    unsigned long getEpochTime() const;

    /**
     * Fills *tv with time in seconds and micros since Jan. 1, 1970
     *
     * @return true if time has been set, else false
     */
    bool getTimeOfDay(NTPTimeval *tv) const;

    /**
     * Stops the underlying UDP client
     */
    void end();
};
