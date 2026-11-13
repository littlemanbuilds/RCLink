#pragma once
#include <cstdint>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>
#define F(s) s
#define INPUT 0
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
    std::uint32_t next=5000;
    /**
     * @brief Accept mock UART initialization.
     *
     * @param baud Unused requested baud rate.
     */
    void begin(unsigned long baud) { (void)baud; }
    /**
     * @brief Expose bytes only when their scheduled packet is due.
     *
     * @return Number of bytes currently available.
     */
    int available() override { return packet<packets.size() && millis()>=next ? static_cast<int>(packets[packet].size()-offset) : 0; }
    /**
     * @brief Consume a byte and schedule the next packet.
     *
     * @return Unsigned byte as int, or minus one when idle.
     */
    int read() override {
        if(!available()) return -1;
        int v=packets[packet][offset++];
        if(offset==packets[packet].size()) { ++packet; offset=0; next+=2; }
        return v;
    }
};
struct Monitor {
    int phase=0;
    std::uint32_t stop=5200;
    std::string output;
    /**
     * @brief Simulate begin and stop keys at the configured times.
     *
     * @return One when an unread key is available, otherwise zero.
     */
    int available() { return phase==0 || (phase==1 && millis()>=stop); }
    /**
     * @brief Consume one simulated monitor key.
     *
     * @return Zero as the mock key byte.
     */
    int read() { ++phase; return 0; }
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
