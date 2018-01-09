/*

CDC Arduino Library by Ihsan Kehribar (kehribar.me)
and Digistump LLC (digistump.com)
- all changes made under the same license as V-USB


 */
#ifndef __DigiWebUSB_h__
#define __DigiWebUSB_h__
#include "usbdrv.h"

#include "Stream.h"
#include "ringBuffer.h"

#define HW_CDC_TX_BUF_SIZE 32
#define HW_CDC_RX_BUF_SIZE 32
#define HW_CDC_BULK_OUT_SIZE 8
#define HW_CDC_BULK_IN_SIZE 8
#define USB_BOS_DESCRIPTOR_TYPE		15
#define WEBUSB_REQUEST_GET_ALLOWED_ORIGINS		0x01
#define WEBUSB_REQUEST_GET_URL			0x02

#define MS_OS_20_REQUEST_DESCRIPTOR 0x07
typedef struct
{
        uint8_t len;         // 9
        uint8_t dtype;       // 4
        uint8_t number;
        uint8_t alternate;
        uint8_t numEndpoints;
        uint8_t interfaceClass;
        uint8_t interfaceSubClass;
        uint8_t protocol;
        uint8_t iInterface;
} InterfaceDescriptor;

//      Endpoint
typedef struct
{
        uint8_t len;         // 7
        uint8_t dtype;       // 5
        uint8_t addr;
        uint8_t attr;
        uint16_t packetSize;
        uint8_t interval;
} EndpointDescriptor;

typedef struct
{
	InterfaceDescriptor dif;
	EndpointDescriptor  in;
	EndpointDescriptor  out;
} WebUSBDescriptor;

typedef struct
{
  uint8_t scheme;
  const char* url;
} WebUSBURL;

/* library functions and variables start */
static uint8_t tmp[HW_CDC_BULK_IN_SIZE];
static uint8_t index = 0;

static RingBuffer_t rxBuf;
static uint8_t rxBuf_Data[HW_CDC_RX_BUF_SIZE];

static RingBuffer_t txBuf;
static uint8_t txBuf_Data[HW_CDC_TX_BUF_SIZE];

class DigiWebUSBDevice : public Stream {
public:
  DigiWebUSBDevice(const WebUSBURL *urls, uint8_t numUrls, uint8_t landingPage,
                   const uint8_t *allowedOrigins, uint8_t numAllowedOrigins);

  void begin(), begin(unsigned long x);
  void end();
  void refresh();
  void task();
  void delay(long milli);
  virtual int available(void);
  virtual int peek(void);
  virtual int read(void);
  virtual void flush(void);
  virtual size_t write(uint8_t);
  using Print::write;
  operator bool();

private:
  void usbBegin();
  void usbPollWrapper();
  const WebUSBURL *urls;
  uint8_t numUrls;
  uint8_t landingPage;
  const uint8_t *allowedOrigins;
  uint8_t numAllowedOrigins;
};



#endif // __DigiWebUSB_h__
