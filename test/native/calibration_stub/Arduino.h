#pragma once
#include <cstdint>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>
#define F(s) s
#define INPUT 0
#define SERIAL_8N1 1
#define SERIAL_8E2 2
/**
 * @brief Access the mock wrapping millisecond clock.
 *
 * @return Mutable clock shared by all mock devices.
 */
inline std::uint32_t &clock_ms() { static std::uint32_t t=0; return t; }
/**
 * @brief Read simulated time.
 *
 * @return Current mock millisecond timestamp.
 */
inline std::uint32_t millis() { return clock_ms(); }
/**
 * @brief Advance simulated time.
 *
 * @param ms Milliseconds to advance.
 */
inline void delay(unsigned ms) { clock_ms()+=ms; }
/**
 * @brief Ignore mock GPIO setup.
 *
 * @param pin Unused simulated pin.
 * @param mode Unused simulated mode.
 */
inline void pinMode(int pin,int mode) { (void)pin; (void)mode; }
struct Stream {
    /**
     * @brief Destroy the mock stream.
     */
    virtual ~Stream() {}
    /**
     * @brief Report available mock bytes.
     *
     * @return Zero for the base stream.
     */
    virtual int available() { return 0; }
    /**
     * @brief Read a mock byte.
     *
     * @return Minus one for the base stream.
     */
    virtual int read() { return -1; }
};
struct HardwareSerial : Stream {
    std::vector<std::vector<std::uint8_t>> packets;
    std::size_t packet=0, offset=0;
    std::uint32_t next=5000, step=2;
    unsigned long configuredBaud=0;
    int configuredFraming=0, configuredRx=-1, configuredTx=-1;
    bool configuredInverted=false;
    /**
     * @brief Accept mock UART initialization.
     *
     * @param baud Unused requested baud rate.
     */
    void begin(unsigned long baud) { configuredBaud=baud; }
    /**
     * @brief Record ESP32 UART initialization without modifying packet data.
     *
     * @param baud Requested baud rate.
     * @param framing Requested UART framing.
     * @param rx Requested RX pin.
     * @param tx Requested TX pin.
     * @param inverted Requested inversion.
     */
    void begin(unsigned long baud, int framing, int rx, int tx, bool inverted=false) {
        configuredBaud=baud; configuredFraming=framing; configuredRx=rx;
        configuredTx=tx; configuredInverted=inverted;
    }
    /**
     * @brief Expose bytes only when their scheduled packet is due.
     *
     * @return Number of bytes currently available.
     */
    int available() override { return packet<packets.size() && static_cast<std::int32_t>(millis()-next)>=0 ? static_cast<int>(packets[packet].size()-offset) : 0; }
    /**
     * @brief Consume a byte and schedule the next packet.
     *
     * @return Unsigned byte as int, or minus one when idle.
     */
    int read() override {
        if(!available()) return -1;
        int v=packets[packet][offset++];
        if(offset==packets[packet].size()) { ++packet; offset=0; next+=step; }
        return v;
    }
};
struct Monitor {
    int phase=0;
    std::uint32_t stop=5200;
    std::string output;
    unsigned long baud=0;
    /**
     * @brief Record the console baud configured by the actual example entry point.
     *
     * @param value Console baud rate.
     */
    void begin(unsigned long value) { baud=value; }
    int capacity=32;
    std::size_t maxWrite=0, writeCalls=0;
    /**
     * @brief Simulate begin and stop keys at the configured times.
     *
     * @return One when an unread key is available, otherwise zero.
     */
    int available() { return phase==0 || (phase==1 && static_cast<std::int32_t>(millis()-stop)>=0); }
    /**
     * @brief Consume one simulated monitor key.
     *
     * @return Zero as the mock key byte.
     */
    int read() { ++phase; return 0; }
    /**
     * @brief Match Arduino Print's numeric unsigned-byte overload.
     *
     * @param value Byte rendered as a decimal number, not a character.
     */
    void print(unsigned char value) { output+=std::to_string(static_cast<unsigned>(value)); }
    /**
     * @brief Append a printable value to captured monitor output.
     *
     * @tparam T Printable argument type.
     * @param v Value to print.
     */
    template<class T> void print(T v) { std::ostringstream s; s<<v; output+=s.str(); }
    /**
     * @brief Append a printable value to captured monitor output.
     *
     * @tparam T Printable argument type.
     * @param v Value to print.
     */
    template<class T> void println(T v) { print(v); output+='\n'; }
    /**
     * @brief Append a monitor newline.
     */
    void println() { output+='\n'; }
    /**
     * @brief Report mock transmit capacity.
     *
     * @return Thirty-two bytes available for each bounded write.
     */
    int availableForWrite() { return capacity; }
    /**
     * @brief Capture a bounded transmit chunk.
     *
     * @param data Bytes to capture.
     * @param size Number of bytes.
     * @return Number of bytes captured.
     */
    std::size_t write(const std::uint8_t *data, std::size_t size) {
        if (size>maxWrite) maxWrite=size;
        ++writeCalls; output.append(reinterpret_cast<const char *>(data),size); return size;
    }
    /**
     * @brief Append literal monitor text.
     *
     * @param s Null-terminated literal text.
     */
    void printf(const char *s) { output+=s; }
    /**
     * @brief Format bounded mock monitor output.
     *
     * @tparam T Printable argument type.
     * @tparam Args Remaining printf argument types.
     * @param s Printf format.
     * @param v First format argument.
     * @param args Remaining format arguments.
     */
    template<class T, class... Args> void printf(const char *s,T v,Args... args) {
        char text[1024]; std::snprintf(text,sizeof(text),s,v,args...); output+=text;
    }
};
extern Monitor Serial;
