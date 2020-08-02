#include "dim_curve.h"
#include <stdint.h>
#include <avr/pgmspace.h>

void getRGB(uint16_t hue, uint8_t val, uint8_t colors[3]) {
  /* convert hue, saturation and brightness ( HSB/HSV ) to RGB
     The dim_curve is used only on brightness/value and on saturation (inverted).
     This looks the most natural.
  */
  uint8_t r;
  uint8_t g;
  uint8_t b;

  if(val == 0) {
    r = 0;
    g = 0;
    b = 0;
  } else {
    val = pgm_read_byte(&dim_curve[val]);
    hue = hue % 360;

    switch(hue/60) {
      case 0:
          r = val;
          g = ((val*hue)/60);
          b = 0;
      break;

      case 1:
          r = ((val*(60-(hue%60)))/60);
          g = val;
          b = 0;
      break;

      case 2:
          r = 0;
          g = val;
          b = ((val*(hue%60))/60);
      break;

      case 3:
          r = 0;
          g = ((val*(60-(hue%60)))/60);
          b = val;
      break;

      case 4:
          r = ((val*(hue%60))/60);
          g = 0;
          b = val;
      break;

      case 5:
          r = val;
          g = 0;
          b = ((val*(60-(hue%60)))/60);
      break;
    }
  }

  colors[0]=r;
  colors[1]=g;
  colors[2]=b;
}
