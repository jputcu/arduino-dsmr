/**
 * Arduino DSMR parser.
 *
 * This software is licensed under the MIT License.
 *
 * Copyright (c) 2015 Matthijs Kooijman <matthijs@stdin.nl>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT.  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * P1 reader, that takes care of toggling a request pin, reading data
 * from a serial port and parsing it.
 */

#ifndef DSMR_INCLUDE_READER_H
#define DSMR_INCLUDE_READER_H

#include <Arduino.h>
#include "crc16.h"

#include "parser.h"

namespace dsmr
{

  /**
 * Controls the request pin on the P1 port to enable (periodic)
 * transmission of messages and reads those messages.
 *
 * To enable the request pin, call enable(). This lets the Smart Meter
 * start periodically sending messages. While the request pin is
 * enabled, loop() should be regularly called to read pending bytes.
 *
 * Once a full and correct message is received, loop() (and available())
 * start returning true, until the message is cleared. You can then
 * either read the raw message using raw(), or parse it using parse().
 *
 * The message is cleared when:
 *  - clear() is called
 *  - parse() is called
 *  - loop() is called and the start of a new message is available
 *
 * When disable is called, the request pin is disabled again and any
 * partial message is discarded. Any bytes received while disabled are
 * dropped.
 */
 template<size_t RX_BUF_SIZE>
  class P1Reader
  {
  public:
    /**
     * Create a new P1Reader. The stream passed should be the serial
     * port to which the P1 TX pin is connected. The req_pin is the
     * pin connected to the request pin. The pin is configured as an
     * output, the Stream is assumed to be already set up (e.g. baud
     * rate configured).
     */
    P1Reader(Stream &stream, uint8_t req_pin)
        : stream(stream), req_pin(req_pin)
    {
      pinMode(req_pin, OUTPUT);
      digitalWrite(req_pin, LOW);
    }

    /**
     * Enable the request pin, to request data on the P1 port.
     * @param  once    When true, the request pin is automatically
     *                 disabled once a complete and correct message was
     *                 receivedc. When false, the request pin stays
     *                 enabled, so messages will continue to be sent
     *                 periodically.
     */
    void enable(bool once)
    {
      digitalWrite(this->req_pin, HIGH);
      m_state = State::waiting;
      m_once = once;
    }

    /* Disable the request pin again, to stop data from being sent on
     * the P1 port. This will also clear any incomplete data that was
     * previously received, but a complete message will be kept until
     * clear() is called.
     */
    void disable()
    {
      digitalWrite(this->req_pin, LOW);
      m_state = State::disabled;
      if (!this->m_msg)
        this->m_write_ptr = m_buffer;
      // Clear any pending bytes
      while (this->stream.read() >= 0) /* nothing */
        ;
    }

    /**
     * Returns true when a complete and correct message was received,
     * until it is cleared.
     */
    bool available()
    {
      return this->m_msg;
    }

    /**
     * Check for new data to read. Should be called regularly, such as
     * once every loop. Returns true if a complete message is available
     * (just like available).
     */
    bool loop()
    {
      while (true)
      {
        if (m_state == State::checksum)
        {
          // Let the Stream buffer the CRC bytes. Convert to size_t to
          // prevent unsigned vs signed comparison
          if ((size_t)this->stream.available() < CrcParser::CRC_LEN)
            return false;

          char buf[CrcParser::CRC_LEN];
          for (auto &c : buf)
            c = static_cast<char>(this->stream.read());

          const auto rcvd_crc = CrcParser::parse(buf, buf + lengthof(buf));

          // Prepare for next message
          m_state = State::waiting;

          if (!rcvd_crc.err && (rcvd_crc.result == m_calc_crc))
          {
            // Message complete, checksum correct
            *this->m_write_ptr++ = '\0';
            this->m_msg = m_buffer;

            if (m_once)
              this->disable();

            return true;
          }
        }
        else
        {
          // For other states, read bytes one by one
          auto c_i = this->stream.read();
          if (c_i < 0)
            return false;
          auto c = static_cast<char>(c_i);

          if( m_state == State::reading ) {
            // Include the ! in the CRC
            m_calc_crc = _crc16_update(m_calc_crc, c);
            if (c != '!') {
              if (!IsFull())
                *m_write_ptr++ = c;
              else
                m_state = State::waiting; // safe state
            } else
              m_state = State::checksum;
          } else if( m_state == State::waiting ) {
            if (c == '/')
            {
              m_state = State::reading;
              // Include the / in the CRC
              m_calc_crc = _crc16_update(0, c);
              this->clear();
            }
          }
        }
      }
      return false;
    }

    /**
     * Returns the data read so far.
     */
    const char* raw()
    {
      return m_buffer;
    }

    bool IsFull() const { return m_write_ptr == m_buffer + RX_BUF_SIZE; }

    /**
     * If a complete message has been received, parse it and store the
     * result into the ParsedData object passed.
     *
     * After parsing, the message is cleared.
     *
     * If parsing fails, false is returned. If err is passed, the error
     * message is appended to that string.
     */
    template <typename... Ts>
    ParseResult<void> parse(ParsedData<Ts...> &data, String *err)
    {
      ParseResult<void> res;
      if( !IsFull() )
        res = P1Parser::parse_data(data, m_buffer, m_write_ptr);
      else
        res.fail(Error::buffer_overflow);

      if (res.err && err)
        *err = res.fullError(m_buffer, m_write_ptr);

      // Clear the message
      this->clear();

      return res;
    }

    /**
     * Clear any complete message from the buffer.
     */
    void clear()
    {
      m_write_ptr = m_buffer;
      m_msg = nullptr;
    }

  protected:
    Stream &stream;
    uint8_t req_pin;
    enum class State : uint8_t
    {
      disabled,
      waiting,
      reading,
      checksum,
    };
    bool m_once = false;
    State m_state = State::disabled;
    char m_buffer[RX_BUF_SIZE];
    char* m_write_ptr = m_buffer;
    const char* m_msg = nullptr;
    uint16_t m_calc_crc;
  };

} // namespace dsmr

#endif // DSMR_INCLUDE_READER_H
