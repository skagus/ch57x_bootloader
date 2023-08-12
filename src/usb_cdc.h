#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05
#define USB_IAD_DESCRIPTOR_TYPE                 0x0B
#if 0
#define USB_HUB_DESCRIPTOR_TYPE                 0x29
#define USB_HID_DESCRIPTOR_TYPE                 0x21
#define USB_REPORT_DESCRIPTOR_TYPE              0x22
#define USB_DESC_TYPE_REPORT                    0x22
#endif
#define USB_FUCNTION_DESCRIPTOR_TYPE            0x24

#define IDX_LANGUAGE_DESCRIPTOR_TYPE			(0x0)
#define IDX_MANUFACTURER_DESCRIPTOR_TYPE		(0x1)
#define IDX_PRODUCT_DESCRIPTOR_TYPE				(0x2)
#define IDX_SERIAL_DESCRIPTOR_TYPE				(0x3)

#define  SET_LINE_CODING						(0x20)	// Configures DTE rate, stop-bits, parity, and number-of-character
#define  GET_LINE_CODING						(0x21)	// This request allows the host to find out the currently configured line coding.
#define  SET_CONTROL_LINE_STATE					(0x22)	// This request generates RS-232/V.24 style control signals.

const uint8_t gDevDesc_CDC_ACM2 [18] = {
	///////////////////////////////////////
	/// device descriptor
	///////////////////////////////////////
	0x12,                        /* bLength */
	USB_DEVICE_DESCRIPTOR_TYPE,  /* bDescriptorType */
	0x00, 0x02,                  /* bcdUSB: version of USB for this device*/
	0xef,                        /* bDeviceClass : USB.org */
	0x02,                        /* bDeviceSubClass : USB.org */
	0x01,                        /* bDeviceProtocol : USB.org */
	DEFAULT_ENDP0_SIZE,          /* bMaxPacketSize: max packet size of EP0 */
	0x86, 0x1a,                  /* idVendor */
	0x23, 0x57,                  /* idProduct */
	0x00, 0x01,                  /* bcdDevice : Device Version number*/
	IDX_MANUFACTURER_DESCRIPTOR_TYPE,                  /* iManufacturer */
	IDX_PRODUCT_DESCRIPTOR_TYPE,                       /* iProduct */
	IDX_SERIAL_DESCRIPTOR_TYPE,                        /* iSerial */
	0x01,                        /* bNumConfigurations: Number of config desc */
};

const uint8_t gCfgDesc_CDC_ACM2 [0x8d] = { // $$$
	///////////////////////////////////////
	/// config descriptor
	///////////////////////////////////////
	0x09,                              /* bLength */
	USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType */
	0x8d,                              /* $$$ wTotalLength Low */
	0x00,                              /* wTotalLength High */
	0x04,                              /* $$$ bNumInterfaces */  
	0x01,                              /* bConfigurationValue */
	0x00,                              /* iConfiguration */
	0x80,                              /* bmAttributes */
	0x64,                              /* bMaxPower */

#if 1  // 1st interface.
	///////////////////////////////////////
	/// interface association descriptor
	///////////////////////////////////////
	0x08,                              /* bLength */
	USB_IAD_DESCRIPTOR_TYPE,           /* bDescriptorType */
	0x00,                              /* bFirstInterface */
	0x02,                              /* bInterfaceCount */
	0x02,                              /* bFunctionClass */
	0x02,                              /* bFunctionSubClass */
	0x01,                              /* bFunctionProtocol */
	0x00,                              /* iFunction */

	///////////////////////////////////////
	/// interface descriptor
	///////////////////////////////////////
	0x09,                              /* bLength */
	USB_INTERFACE_DESCRIPTOR_TYPE,     /* bDescriptorType */
	0x00,                              /* bInterfaceNumber */
	0x00,                              /* bAlternateSetting */
	0x01,                              /* bNumEndpoints */
	0x02,                              /* bInterfaceClass */
	0x02,                              /* bInterfaceSubClass */
	0x01,                              /* bInterfaceProtocol */
	0x00,                              /* iInterface */

	///////////////////////////////////////
	/// cdc acm header descriptor
	///////////////////////////////////////
	0x05,                              /* bLength */
	USB_FUCNTION_DESCRIPTOR_TYPE,      /* bDescriptorType */
	0x00,                              /* bDescriptorSubtype */
	0x10,                              /* bcdCDC */
	0x01,                              /* bcdCDC */

	///////////////////////////////////////
	/// cdc acm call management descriptor
	///////////////////////////////////////
	0x05,                              /* bLength */
	USB_FUCNTION_DESCRIPTOR_TYPE,      /* bDescriptorType */
	0x01,                              /* bDescriptorSubtype */
	0x00,                              /* bmCapabilities */
	0x01,                              /* bDataInterface */

	///////////////////////////////////////
	/// cdc acm descriptor
	///////////////////////////////////////
	0x04,                              /* bLength */
	USB_FUCNTION_DESCRIPTOR_TYPE,      /* bDescriptorType */
	0x02,                              /* bDescriptorSubtype */
	0x02,                              /* bmCapabilities */

	///////////////////////////////////////
	/// cdc acm union descriptor
	///////////////////////////////////////
	0x05,                              /* bLength */
	USB_FUCNTION_DESCRIPTOR_TYPE,      /* bDescriptorType */
	0x06,                              /* bDescriptorSubtype */
	0x00,                              /* bMasterInterface */
	0x01,                              /* bSlaveInterface0 */

	///////////////////////////////////////
	/// endpoint descriptor
	///////////////////////////////////////
	0x07,                              /* bLength */
	USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType */
	0x85,                              /* bEndpointAddress */
	0x03,                              /* bmAttributes */
	0x08, 0x00,                        /* wMaxPacketSize */
	0x01,                              /* bInterval */

	///////////////////////////////////////
	/// interface descriptor
	///////////////////////////////////////
	0x09,                              /* bLength */
	USB_INTERFACE_DESCRIPTOR_TYPE,     /* bDescriptorType */
	0x01,                              /* bInterfaceNumber */
	0x00,                              /* bAlternateSetting */
	0x02,                              /* bNumEndpoints */
	0x0a,                              /* bInterfaceClass */
	0x00,                              /* bInterfaceSubClass */
	0x00,                              /* bInterfaceProtocol */
	0x00,                              /* iInterface */

	///////////////////////////////////////
	/// endpoint descriptor
	///////////////////////////////////////
	0x07,                              /* bLength */
	USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType */
	0x82,                              /* bEndpointAddress */
	0x02,                              /* bmAttributes */
	0x40, 0x00,                        /* wMaxPacketSize */
	0x01,                              /* bInterval */

	///////////////////////////////////////
	/// endpoint descriptor
	///////////////////////////////////////
	0x07,                              /* bLength */
	USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType */
	0x02,                              /* bEndpointAddress */
	0x02,                              /* bmAttributes */
	0x40, 0x00,                        /* wMaxPacketSize */
	0x01,                              /* bInterval */
#endif

#if 1   // 2nd Interface.
	///////////////////////////////////////
	/// interface association descriptor
	///////////////////////////////////////
	0x08,                              /* bLength */
	USB_IAD_DESCRIPTOR_TYPE,           /* bDescriptorType */
	0x02,                              /* bFirstInterface */
	0x02,                              /* bInterfaceCount */
	0x02,                              /* bFunctionClass */
	0x02,                              /* bFunctionSubClass */
	0x01,                              /* bFunctionProtocol */
	0x00,                              /* iFunction */

	///////////////////////////////////////
	/// interface descriptor
	///////////////////////////////////////
	0x09,                             /* bLength */
	USB_INTERFACE_DESCRIPTOR_TYPE,    /* bDescriptorType */
	0x02,                             /* bInterfaceNumber */
	0x00,                             /* bAlternateSetting */
	0x01,                             /* bNumEndpoints */
	0x02,                             /* bInterfaceClass */
	0x02,                             /* bInterfaceSubClass */
	0x01,                             /* bInterfaceProtocol */
	0x00,                             /* iInterface */

	///////////////////////////////////////
	/// cdc acm header descriptor
	///////////////////////////////////////
	0x05,                             /* bLength */
	USB_FUCNTION_DESCRIPTOR_TYPE,     /* bDescriptorType */
	0x00,                             /* bDescriptorSubtype */
	0x10,                             /* bcdCDC */
	0x01,                             /* bcdCDC */

	///////////////////////////////////////
	/// cdc acm call management descriptor
	///////////////////////////////////////
	0x05,                             /* bLength */
	USB_FUCNTION_DESCRIPTOR_TYPE,     /* bDescriptorType */
	0x01,                             /* bDescriptorSubtype */
	0x00,                             /* bmCapabilities */
	0x01,                             /* bDataInterface */

	///////////////////////////////////////
	/// cdc acm descriptor
	///////////////////////////////////////
	0x04,                             /* bLength */
	USB_FUCNTION_DESCRIPTOR_TYPE,     /* bDescriptorType */
	0x02,                             /* bDescriptorSubtype */
	0x02,                             /* bmCapabilities */

	///////////////////////////////////////
	/// cdc acm union descriptor
	///////////////////////////////////////
	0x05,                            /* bLength */
	USB_FUCNTION_DESCRIPTOR_TYPE,    /* bDescriptorType */
	0x06,                            /* bDescriptorSubtype */
	0x02,                            /* bMasterInterface */
	0x03,                            /* bSlaveInterface0 */

	///////////////////////////////////////
	/// endpoint descriptor
	///////////////////////////////////////
	0x07,                            /* bLength */
	USB_ENDPOINT_DESCRIPTOR_TYPE,    /* bDescriptorType */
	0x86,                            /* bEndpointAddress */
	0x03,                            /* bmAttributes */
	0x08, 0x00,                      /* wMaxPacketSize */
	0x01,                            /* bInterval */

	///////////////////////////////////////
	/// interface descriptor
	///////////////////////////////////////
	0x09,                            /* bLength */
	USB_INTERFACE_DESCRIPTOR_TYPE,   /* bDescriptorType */
	0x03,                            /* bInterfaceNumber */
	0x00,                            /* bAlternateSetting */
	0x02,                            /* bNumEndpoints */
	0x0a,                            /* bInterfaceClass */
	0x00,                            /* bInterfaceSubClass */
	0x00,                            /* bInterfaceProtocol */
	0x00,                            /* iInterface */

	///////////////////////////////////////
	/// endpoint descriptor
	///////////////////////////////////////
	0x07,                            /* bLength */
	USB_ENDPOINT_DESCRIPTOR_TYPE,    /* bDescriptorType */
	0x83,                            /* bEndpointAddress */
	0x02,                            /* bmAttributes */
	0x40, 0x00,                      /* wMaxPacketSize */
	0x01,                            /* bInterval */

	///////////////////////////////////////
	/// endpoint descriptor
	///////////////////////////////////////
	0x07,                            /* bLength */
	USB_ENDPOINT_DESCRIPTOR_TYPE,    /* bDescriptorType */
	0x03,                            /* bEndpointAddress */
	0x02,                            /* bmAttributes */
	0x40, 0x00,                      /* wMaxPacketSize */
	0x01,                            /* bInterval */
#endif
};

const uint8_t gLangDesc [0x04] = {
	0x04,                                         /* bLength */
	USB_STRING_DESCRIPTOR_TYPE,                   /* bDescriptorType */
	0x09, 0x04,                                   /* wLangID0 : Englinsh */
};

const uint8_t gszManuDesc [0x18] = {	// "dog2nd_Zhao"
	0x18,                                             /* bLength */
	USB_STRING_DESCRIPTOR_TYPE,                       /* bDescriptorType */
	'd', 0x00,'o', 0x00,'g', 0x00,'2', 0x00,'n', 0x00,'d', 0x00,'_', 0x00,'Z', 0x00,'h', 0x00,'a', 0x00,'o', 0x00
};

const uint8_t gszProdDesc [0x14] = {	// "USB CBCx2"
	0x14,                                             /* bLength */
	USB_STRING_DESCRIPTOR_TYPE,                       /* bDescriptorType */
	'U', 0x00,'S', 0x00,'B', 0x00,' ', 0x00,'C', 0x00,'D', 0x00,'C', 0x00,'x', 0x00,'2', 0x00
};

const uint8_t gszSerialDesc [0x14] = {	// "USB123456"
	0x14,                                             /* bLength */
	USB_STRING_DESCRIPTOR_TYPE,                       /* bDescriptorType */
	'U', 0x00,'S', 0x00,'B', 0x00,'1', 0x00,'2', 0x00,'3', 0x00,'4', 0x00,'5', 0x00,'6', 0x00,
};

const uint8_t* aToHex = "0123456789ABCDEF";

////////////////////////////////////////////////////////////////////////////

void usb_isr_call();
void uart0_isr_call();
void uart1_isr_call();


void CDC_Run();
void CDC_Init();

