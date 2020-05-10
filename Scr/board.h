#ifndef BOARD_H
#define BOARD_H

/* GPIO Pin identifier */
typedef struct _io_pin_t {
    GPIO_TypeDef *port;
    uint16_t      pin;
} io_pin_t;

// Control USB Power connections
#define PORT_PWR_USB_1                      GPIOA
#define PIN_PWR_USB_1                       GPIO_PIN_11

#define PORT_PWR_USB_2                      GPIOH
#define PIN_PWR_USB_2                       GPIO_PIN_13

#define PORT_PWR_USB_3                      GPIOH
#define PIN_PWR_USB_3                       GPIO_PIN_15

#define PORT_PWR_USB_4                      GPIOI
#define PIN_PWR_USB_4                       GPIO_PIN_2

static const io_pin_t IO_PIN_PW_USB[]={
    {PORT_PWR_USB_1, PIN_PWR_USB_1},
    {PORT_PWR_USB_2, PIN_PWR_USB_2},
    {PORT_PWR_USB_3, PIN_PWR_USB_3},
    {PORT_PWR_USB_4, PIN_PWR_USB_4}
};

#define PORT_RST_HUB                        GPIOF
#define PIN_RST_HUB                         GPIO_PIN_2

#endif // BOARD_H

