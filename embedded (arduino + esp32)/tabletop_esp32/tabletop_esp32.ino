
//#include <Arduino.h>
//#include <WiFi.h>
//#include <FreeRTOS.h>
//#include <AsyncTCP.h>
#include "My_Adafruit_DotStar.h"
#include <ESPAsyncWebSrv.h>
#include <ArduinoJson.h>
#include <xtensa/core-macros.h>
#include <QuickStats.h>
#include <math.h>

#define RXD2 16
#define TXD2 17
#define SERIAL_BAUDRATE 115200
#define SPI_FREQUENCY 20000000
#define DEBUG_CMD "debug "
#define BRIGHTNESS_CMD "brightness"
#define RESET_CMD "reset"
#define ROTATION_CMD "rotation"
#define ANCHOR_CMD "anchor"
#define ANCHOR_BIS_CMD "anchor_bis"
#define TOP_CMD "top"
#define RIGHT_CMD "right"
#define BOTTOM_CMD "bottom"
#define LEFT_CMD "left"
#define CROP_CMD "crop"
#define CROP_BIS_CMD "crop_bis"
#define OFFSET_CMD "offset"
#define OFFSET_BIS_CMD "offset_bis"
#define X_CMD "x"
#define Y_CMD "y"
#define COLUMN_CORRECTION_CMD "column_correction"
#define FIRST_CMD "first"
#define SECOND_CMD "second"
#define START_CALIBRATION_CMD "start_calibration"
#define STOP_CALIBRATION_CMD "stop_calibration"
#define DISPLAY_HORIZONTAL_RESOLUTION "display_horizontal_resolution"
#define DISPLAY_VERTICAL_RESOLUTION "display_vertical_resolution"
#define DISPLAY_HORIZONTAL_RESOLUTION "display_horizontal_resolution"
#define DISPLAY_ERROR "display_error"
#define EXPERIMENTATION "experimentation"
#define ANGULAR_SPEED "angular_speed"
#define ERROR_WITH_PITCH "error_with_pitch"
#define ERROR_WITHOUT_PITCH "error_without_pitch"
#define TARGET_DELTATIME_SPACING "target_deltatime_spacing"
#define HORIZONTAL_RESOLUTION "horizontal_resolution"
#define VERTICAL_RESOLUTION "vertical_resolution"
#define HEIGHT "height"
#define LENGTH "length"
#define DELTATIME_SPACING_WITH_PITCH "deltatime_spacing_with_pitch"
#define DELTATIME_SPACING_WITHOUT_PITCH "deltatime_spacing_without_pitch"

#define LINE_STATUS "status_line"
#define LINE_CURRENT "current_line"
#define LINE_TARGET "target_line"

#define HEIGHT_STATUS "status_height"
#define HEIGHT_CURRENT "current_height"
#define HEIGHT_TARGET "target_height"

#define SPIN_STATUS "status_spin"
#define SPIN_CURRENT "current_spin"
#define SPIN_TARGET "target_spin"

#define LINE_STATUS "status_line"
#define LINE_CURRENT "current_line"
#define LINE_TARGET "target_line"

#define HORIZONTAL_RESOLUTION_CMD "horizontal_resolution"
#define DISPLAY_RPM_CMD "display_rpm"
#define ENABLE_CMD "enable"
#define DISABLE_CMD "disable"
#define ENABLES_CMD "enables"
#define DISABLES_CMD "disables"
#define PITCH_ENABLE_CMD "pitch_enable"
#define PITCH_DISABLE_CMD "pitch_disable"
#define DUAL_ENABLES_CMD "dual_enables"
#define DUAL_DISABLES_CMD "dual_disables"
#define STATE_CMD "state"
#define SPIN_CMD "spin"
#define SPIN_CMD_CMD "spinCmd"
#define HEIGHT_CMD "height"
#define LINE_CMD "line"
#define LINES_CMD "lines"
#define LINES_INDEX_CMD "index"
#define LINES_POSITION_CMD "position"
#define LINES_DURATION_CMD "duration"
#define DURATION_CMD "duration"
#define INDEX_CMD "index"
#define RESET_CMD "reset"
#define COLOR_CMD "color"
#define LIGHT_CMD "light"
#define LIGHTS_CMD "lights"

#define TOPIC "topic"
#define PAYLOAD "payload"


#define SPINNER_KEEP_ALIVE_DELAY 5000
#define SPINNER_REFRESH_STATE_DELAY 1000

// light parameters

#define VSPI_MISO   MISO // 19
#define VSPI_MOSI   MOSI // 23 
#define VSPI_SCLK   SCK // 18
#define VSPI_SS     SS // 5
#define VSPI_MUX 4

#define HSPI_MISO   12  
#define HSPI_MOSI   13
#define HSPI_SCLK   14
#define HSPI_SS     15
#define HSPI_MUX    21

#define MUX_NANOSECONDS_DELAY 200
#define LIGHT_NANOSECONDS_DELAY 50

#define NUMPIXELS    144 // (cut one) otherwise 144
#define NUMPIXELS0     140 // OK  140 
#define NUMPIXELS1     138 // OK  140
#define NUMPIXELS2     142 //   OK 142
#define NUMPIXELS3     142 //   OK 142 
#define NBSTRIPS     4 // (cut one) otherwise 144

#define RESETPIXEL    31
#define RESETPIXELOFFSET  1
#define PITCHPIXEL    7
#define MAXBRIGHTNESS 16 //128
#define MAXAMPPIXEL   60
#define MAXAMP        4000

#define RPM_TO_FPS_ERROR 1.05
#define DISPLAY_DELAY 1000

// INSERT PRE-COMPUTED MAXIMAL HORIZONTAL RESOLUTION FOR EACH TROPOSKEIN
    
#define DISPLAY_MAX_HORIZONTAL_RESOLUTION 425
#define DISPLAY_MAX_VERTICAL_RESOLUTION 116
#define DISPLAY_MAX_NB_PIXELS 55000//49300
#define MIN_H 200
#define MAX_H 501
#define STEP_H 7
#define NB_H 44
#define MIN_L 217
#define MAX_L 812
#define STEP_L 7
#define NB_L 86
#define NB_MAX_HORIZONTAL_RESOLUTIONS 2923
uint16_t max_horizontal_resolutions [NB_MAX_HORIZONTAL_RESOLUTIONS] = {119, 125, 131, 137, 142, 147, 152, 156, 161, 165, 169, 173, 177, 181, 185, 189, 193, 197, 201, 204, 208, 212, 215, 219, 222, 226, 229, 233, 237, 240, 243, 247, 250, 254, 257, 261, 264, 267, 271, 274, 278, 281, 284, 288, 291, 294, 298, 301, 304, 308, 311, 314, 318, 321, 324, 328, 331, 334, 337, 341, 344, 347, 351, 354, 357, 360, 364, 367, 370, 373, 377, 380, 383, 386, 390, 393, 396, 399, 403, 406, 409, 412, 415, 419, 422, 425, 111, 119, 126, 132, 138, 143, 148, 153, 157, 162, 166, 170, 175, 179, 183, 187, 191, 194, 198, 202, 206, 209, 213, 217, 220, 224, 227, 231, 235, 238, 242, 245, 249, 252, 256, 259, 262, 266, 269, 273, 276, 279, 283, 286, 290, 293, 296, 300, 303, 306, 310, 313, 316, 320, 323, 326, 329, 333, 336, 339, 343, 346, 349, 352, 356, 359, 362, 366, 369, 372, 375, 379, 382, 385, 388, 392, 395, 398, 401, 405, 408, 411, 414, 418, 421, 424, 99, 111, 120, 127, 133, 139, 144, 149, 154, 158, 163, 167, 172, 176, 180, 184, 188, 192, 196, 200, 203, 207, 211, 215, 218, 222, 225, 229, 233, 236, 240, 243, 247, 250, 254, 257, 261, 264, 267, 271, 274, 278, 281, 285, 288, 291, 295, 298, 301, 305, 308, 311, 315, 318, 321, 325, 328, 331, 335, 338, 341, 345, 348, 351, 354, 358, 361, 364, 368, 371, 374, 377, 381, 384, 387, 390, 394, 397, 400, 403, 407, 410, 413, 416, 420, 423, 99, 112, 120, 127, 134, 139, 145, 150, 155, 160, 164, 168, 173, 177, 181, 185, 189, 193, 197, 201, 205, 208, 212, 216, 220, 223, 227, 231, 234, 238, 241, 245, 248, 252, 255, 259, 262, 266, 269, 273, 276, 279, 283, 286, 290, 293, 296, 300, 303, 307, 310, 313, 317, 320, 323, 327, 330, 333, 337, 340, 343, 346, 350, 353, 356, 360, 363, 366, 369, 373, 376, 379, 383, 386, 389, 392, 396, 399, 402, 405, 409, 412, 415, 418, 422, 99, 112, 121, 128, 134, 140, 146, 151, 156, 160, 165, 170, 174, 178, 182, 186, 190, 194, 198, 202, 206, 210, 214, 217, 221, 225, 228, 232, 236, 239, 243, 246, 250, 253, 257, 260, 264, 267, 271, 274, 278, 281, 285, 288, 291, 295, 298, 302, 305, 308, 312, 315, 318, 322, 325, 328, 332, 335, 338, 342, 345, 348, 352, 355, 358, 361, 365, 368, 371, 375, 378, 381, 384, 388, 391, 394, 398, 401, 404, 407, 411, 414, 417, 420, 100, 112, 121, 129, 135, 141, 147, 152, 157, 161, 166, 171, 175, 179, 183, 188, 192, 196, 200, 204, 207, 211, 215, 219, 222, 226, 230, 233, 237, 241, 244, 248, 251, 255, 258, 262, 266, 269, 272, 276, 279, 283, 286, 290, 293, 296, 300, 303, 307, 310, 313, 317, 320, 323, 327, 330, 334, 337, 340, 344, 347, 350, 353, 357, 360, 363, 367, 370, 373, 377, 380, 383, 386, 390, 393, 396, 400, 403, 406, 409, 413, 416, 419, 100, 113, 122, 129, 136, 142, 147, 153, 158, 162, 167, 172, 176, 180, 185, 189, 193, 197, 201, 205, 209, 213, 216, 220, 224, 228, 231, 235, 239, 242, 246, 249, 253, 257, 260, 264, 267, 271, 274, 278, 281, 284, 288, 291, 295, 298, 302, 305, 308, 312, 315, 319, 322, 325, 329, 332, 335, 339, 342, 345, 349, 352, 355, 359, 362, 365, 369, 372, 375, 378, 382, 385, 388, 392, 395, 398, 401, 405, 408, 411, 415, 418, 100, 113, 122, 130, 136, 142, 148, 153, 159, 163, 168, 173, 177, 182, 186, 190, 194, 198, 202, 206, 210, 214, 218, 222, 225, 229, 233, 236, 240, 244, 247, 251, 255, 258, 262, 265, 269, 272, 276, 279, 283, 286, 290, 293, 296, 300, 303, 307, 310, 314, 317, 320, 324, 327, 330, 334, 337, 340, 344, 347, 351, 354, 357, 360, 364, 367, 370, 374, 377, 380, 384, 387, 390, 394, 397, 400, 403, 407, 410, 413, 416, 100, 114, 123, 130, 137, 143, 149, 154, 159, 164, 169, 174, 178, 183, 187, 191, 195, 199, 203, 207, 211, 215, 219, 223, 227, 230, 234, 238, 242, 245, 249, 252, 256, 260, 263, 267, 270, 274, 277, 281, 284, 288, 291, 295, 298, 302, 305, 308, 312, 315, 319, 322, 325, 329, 332, 336, 339, 342, 346, 349, 352, 356, 359, 362, 366, 369, 372, 376, 379, 382, 386, 389, 392, 395, 399, 402, 405, 409, 412, 415, 100, 114, 123, 131, 138, 144, 150, 155, 160, 165, 170, 175, 179, 184, 188, 192, 196, 201, 205, 209, 213, 217, 220, 224, 228, 232, 236, 239, 243, 247, 250, 254, 258, 261, 265, 268, 272, 275, 279, 282, 286, 289, 293, 296, 300, 303, 307, 310, 314, 317, 320, 324, 327, 331, 334, 337, 341, 344, 347, 351, 354, 357, 361, 364, 367, 371, 374, 377, 381, 384, 387, 391, 394, 397, 401, 404, 407, 410, 414, 101, 114, 124, 132, 138, 145, 151, 156, 161, 166, 171, 176, 180, 185, 189, 193, 198, 202, 206, 210, 214, 218, 222, 226, 229, 233, 237, 241, 244, 248, 252, 255, 259, 263, 266, 270, 273, 277, 280, 284, 288, 291, 294, 298, 301, 305, 308, 312, 315, 319, 322, 325, 329, 332, 336, 339, 342, 346, 349, 353, 356, 359, 363, 366, 369, 373, 376, 379, 383, 386, 389, 393, 396, 399, 402, 406, 409, 412, 101, 115, 124, 132, 139, 145, 151, 157, 162, 167, 172, 177, 181, 186, 190, 195, 199, 203, 207, 211, 215, 219, 223, 227, 231, 235, 238, 242, 246, 250, 253, 257, 261, 264, 268, 271, 275, 278, 282, 286, 289, 293, 296, 300, 303, 307, 310, 313, 317, 320, 324, 327, 331, 334, 337, 341, 344, 348, 351, 354, 358, 361, 364, 368, 371, 374, 378, 381, 384, 388, 391, 394, 398, 401, 404, 408, 411, 101, 115, 125, 133, 140, 146, 152, 158, 163, 168, 173, 178, 182, 187, 191, 196, 200, 204, 208, 212, 216, 220, 224, 228, 232, 236, 240, 243, 247, 251, 255, 258, 262, 266, 269, 273, 276, 280, 284, 287, 291, 294, 298, 301, 305, 308, 312, 315, 319, 322, 325, 329, 332, 336, 339, 343, 346, 349, 353, 356, 359, 363, 366, 370, 373, 376, 380, 383, 386, 390, 393, 396, 400, 403, 406, 410, 101, 115, 125, 133, 140, 147, 153, 158, 164, 169, 174, 179, 183, 188, 192, 197, 201, 205, 209, 214, 218, 222, 226, 229, 233, 237, 241, 245, 249, 252, 256, 260, 263, 267, 271, 274, 278, 282, 285, 289, 292, 296, 299, 303, 306, 310, 313, 317, 320, 324, 327, 331, 334, 337, 341, 344, 348, 351, 354, 358, 361, 365, 368, 371, 375, 378, 381, 385, 388, 391, 395, 398, 401, 405, 408, 101, 116, 126, 134, 141, 148, 154, 159, 165, 170, 175, 180, 184, 189, 193, 198, 202, 206, 211, 215, 219, 223, 227, 231, 235, 239, 242, 246, 250, 254, 257, 261, 265, 269, 272, 276, 279, 283, 287, 290, 294, 297, 301, 304, 308, 311, 315, 318, 322, 325, 329, 332, 336, 339, 343, 346, 349, 353, 356, 360, 363, 366, 370, 373, 376, 380, 383, 387, 390, 393, 397, 400, 403, 407, 102, 116, 126, 134, 142, 148, 154, 160, 165, 171, 176, 181, 185, 190, 195, 199, 203, 208, 212, 216, 220, 224, 228, 232, 236, 240, 244, 248, 251, 255, 259, 263, 266, 270, 274, 277, 281, 285, 288, 292, 295, 299, 302, 306, 309, 313, 316, 320, 323, 327, 330, 334, 337, 341, 344, 348, 351, 354, 358, 361, 365, 368, 371, 375, 378, 382, 385, 388, 392, 395, 398, 402, 405, 102, 117, 127, 135, 142, 149, 155, 161, 166, 172, 177, 182, 186, 191, 196, 200, 204, 209, 213, 217, 221, 225, 229, 233, 237, 241, 245, 249, 253, 256, 260, 264, 268, 271, 275, 279, 282, 286, 290, 293, 297, 300, 304, 308, 311, 315, 318, 322, 325, 329, 332, 336, 339, 342, 346, 349, 353, 356, 360, 363, 366, 370, 373, 377, 380, 383, 387, 390, 393, 397, 400, 404, 102, 117, 127, 135, 143, 150, 156, 162, 167, 172, 178, 183, 187, 192, 197, 201, 205, 210, 214, 218, 222, 227, 231, 235, 239, 242, 246, 250, 254, 258, 262, 265, 269, 273, 277, 280, 284, 288, 291, 295, 298, 302, 306, 309, 313, 316, 320, 323, 327, 330, 334, 337, 341, 344, 348, 351, 354, 358, 361, 365, 368, 372, 375, 378, 382, 385, 388, 392, 395, 399, 402, 102, 117, 128, 136, 143, 150, 156, 162, 168, 173, 178, 183, 188, 193, 198, 202, 207, 211, 215, 219, 224, 228, 232, 236, 240, 244, 248, 252, 255, 259, 263, 267, 271, 274, 278, 282, 285, 289, 293, 296, 300, 303, 307, 311, 314, 318, 321, 325, 328, 332, 335, 339, 342, 346, 349, 353, 356, 360, 363, 366, 370, 373, 377, 380, 383, 387, 390, 394, 397, 400, 102, 118, 128, 137, 144, 151, 157, 163, 169, 174, 179, 184, 189, 194, 199, 203, 208, 212, 216, 221, 225, 229, 233, 237, 241, 245, 249, 253, 257, 261, 264, 268, 272, 276, 279, 283, 287, 290, 294, 298, 301, 305, 309, 312, 316, 319, 323, 326, 330, 333, 337, 340, 344, 347, 351, 354, 358, 361, 365, 368, 372, 375, 378, 382, 385, 389, 392, 395, 399, 103, 118, 128, 137, 145, 152, 158, 164, 170, 175, 180, 185, 190, 195, 200, 204, 209, 213, 217, 222, 226, 230, 234, 238, 242, 246, 250, 254, 258, 262, 266, 270, 273, 277, 281, 285, 288, 292, 296, 299, 303, 306, 310, 314, 317, 321, 324, 328, 331, 335, 338, 342, 345, 349, 352, 356, 359, 363, 366, 370, 373, 377, 380, 383, 387, 390, 394, 397, 103, 118, 129, 138, 145, 152, 159, 165, 170, 176, 181, 186, 191, 196, 201, 205, 210, 214, 219, 223, 227, 231, 235, 239, 244, 248, 251, 255, 259, 263, 267, 271, 275, 278, 282, 286, 290, 293, 297, 301, 304, 308, 312, 315, 319, 322, 326, 329, 333, 337, 340, 344, 347, 351, 354, 358, 361, 364, 368, 371, 375, 378, 382, 385, 389, 392, 395, 103, 119, 129, 138, 146, 153, 159, 165, 171, 177, 182, 187, 192, 197, 202, 206, 211, 215, 220, 224, 228, 232, 237, 241, 245, 249, 253, 257, 261, 265, 268, 272, 276, 280, 284, 287, 291, 295, 298, 302, 306, 309, 313, 317, 320, 324, 327, 331, 335, 338, 342, 345, 349, 352, 356, 359, 363, 366, 370, 373, 377, 380, 383, 387, 390, 394, 103, 119, 130, 139, 146, 153, 160, 166, 172, 177, 183, 188, 193, 198, 203, 207, 212, 216, 221, 225, 229, 234, 238, 242, 246, 250, 254, 258, 262, 266, 270, 274, 277, 281, 285, 289, 292, 296, 300, 304, 307, 311, 315, 318, 322, 325, 329, 333, 336, 340, 343, 347, 350, 354, 357, 361, 364, 368, 371, 375, 378, 382, 385, 389, 392, 103, 119, 130, 139, 147, 154, 161, 167, 173, 178, 184, 189, 194, 199, 204, 208, 213, 217, 222, 226, 230, 235, 239, 243, 247, 251, 255, 259, 263, 267, 271, 275, 279, 283, 286, 290, 294, 298, 301, 305, 309, 312, 316, 320, 323, 327, 330, 334, 338, 341, 345, 348, 352, 355, 359, 362, 366, 369, 373, 376, 380, 383, 387, 390, 104, 120, 131, 140, 148, 155, 161, 168, 173, 179, 184, 190, 195, 200, 205, 209, 214, 218, 223, 227, 232, 236, 240, 244, 248, 252, 256, 260, 264, 268, 272, 276, 280, 284, 288, 291, 295, 299, 303, 306, 310, 314, 317, 321, 325, 328, 332, 336, 339, 343, 346, 350, 353, 357, 360, 364, 367, 371, 374, 378, 381, 385, 388, 104, 120, 131, 140, 148, 155, 162, 168, 174, 180, 185, 191, 196, 201, 205, 210, 215, 219, 224, 228, 233, 237, 241, 245, 250, 254, 258, 262, 266, 270, 274, 277, 281, 285, 289, 293, 297, 300, 304, 308, 312, 315, 319, 323, 326, 330, 333, 337, 341, 344, 348, 351, 355, 359, 362, 366, 369, 373, 376, 380, 383, 387, 104, 120, 131, 141, 149, 156, 163, 169, 175, 181, 186, 191, 197, 202, 206, 211, 216, 220, 225, 229, 234, 238, 242, 247, 251, 255, 259, 263, 267, 271, 275, 279, 283, 287, 290, 294, 298, 302, 306, 309, 313, 317, 320, 324, 328, 331, 335, 339, 342, 346, 349, 353, 357, 360, 364, 367, 371, 374, 378, 381, 385, 104, 121, 132, 141, 149, 156, 163, 170, 176, 181, 187, 192, 197, 202, 207, 212, 217, 221, 226, 230, 235, 239, 243, 248, 252, 256, 260, 264, 268, 272, 276, 280, 284, 288, 292, 296, 299, 303, 307, 311, 314, 318, 322, 325, 329, 333, 336, 340, 344, 347, 351, 354, 358, 362, 365, 369, 372, 376, 379, 383, 104, 121, 132, 142, 150, 157, 164, 170, 176, 182, 188, 193, 198, 203, 208, 213, 218, 222, 227, 232, 236, 240, 245, 249, 253, 257, 261, 265, 269, 273, 277, 281, 285, 289, 293, 297, 301, 305, 308, 312, 316, 320, 323, 327, 331, 334, 338, 342, 345, 349, 352, 356, 360, 363, 367, 370, 374, 377, 381, 104, 121, 133, 142, 150, 158, 165, 171, 177, 183, 189, 194, 199, 204, 209, 214, 219, 223, 228, 233, 237, 241, 246, 250, 254, 258, 263, 267, 271, 275, 279, 283, 287, 290, 294, 298, 302, 306, 310, 313, 317, 321, 325, 328, 332, 336, 339, 343, 347, 350, 354, 358, 361, 365, 368, 372, 375, 379, 105, 121, 133, 143, 151, 158, 165, 172, 178, 184, 189, 195, 200, 205, 210, 215, 220, 224, 229, 234, 238, 242, 247, 251, 255, 260, 264, 268, 272, 276, 280, 284, 288, 292, 296, 300, 303, 307, 311, 315, 319, 322, 326, 330, 333, 337, 341, 344, 348, 352, 355, 359, 363, 366, 370, 373, 377, 105, 122, 133, 143, 151, 159, 166, 172, 179, 184, 190, 196, 201, 206, 211, 216, 221, 225, 230, 235, 239, 244, 248, 252, 256, 261, 265, 269, 273, 277, 281, 285, 289, 293, 297, 301, 305, 309, 312, 316, 320, 324, 327, 331, 335, 339, 342, 346, 350, 353, 357, 361, 364, 368, 371, 375, 105, 122, 134, 143, 152, 159, 166, 173, 179, 185, 191, 196, 202, 207, 212, 217, 222, 226, 231, 236, 240, 245, 249, 253, 258, 262, 266, 270, 274, 278, 282, 286, 290, 294, 298, 302, 306, 310, 314, 318, 321, 325, 329, 333, 336, 340, 344, 347, 351, 355, 358, 362, 366, 369, 373, 105, 122, 134, 144, 152, 160, 167, 174, 180, 186, 192, 197, 203, 208, 213, 218, 223, 227, 232, 237, 241, 246, 250, 254, 259, 263, 267, 271, 275, 280, 284, 288, 292, 296, 300, 303, 307, 311, 315, 319, 323, 326, 330, 334, 338, 341, 345, 349, 353, 356, 360, 363, 367, 371, 105, 123, 135, 144, 153, 161, 168, 174, 181, 187, 192, 198, 203, 209, 214, 219, 224, 228, 233, 238, 242, 247, 251, 256, 260, 264, 268, 273, 277, 281, 285, 289, 293, 297, 301, 305, 309, 313, 316, 320, 324, 328, 332, 335, 339, 343, 347, 350, 354, 358, 361, 365, 369, 105, 123, 135, 145, 153, 161, 168, 175, 181, 187, 193, 199, 204, 210, 215, 220, 225, 229, 234, 239, 243, 248, 252, 257, 261, 265, 269, 274, 278, 282, 286, 290, 294, 298, 302, 306, 310, 314, 318, 322, 325, 329, 333, 337, 341, 344, 348, 352, 355, 359, 363, 366, 106, 123, 135, 145, 154, 162, 169, 176, 182, 188, 194, 200, 205, 210, 216, 221, 226, 230, 235, 240, 244, 249, 253, 258, 262, 266, 271, 275, 279, 283, 287, 291, 295, 299, 303, 307, 311, 315, 319, 323, 327, 331, 334, 338, 342, 346, 349, 353, 357, 361, 364, 106, 124, 136, 146, 154, 162, 170, 176, 183, 189, 195, 200, 206, 211, 216, 221, 226, 231, 236, 241, 245, 250, 254, 259, 263, 267, 272, 276, 280, 284, 288, 293, 297, 301, 305, 309, 313, 316, 320, 324, 328, 332, 336, 339, 343, 347, 351, 355, 358, 362, 106, 124, 136, 146, 155, 163, 170, 177, 183, 190, 196, 201, 207, 212, 217, 222, 227, 232, 237, 242, 246, 251, 255, 260, 264, 269, 273, 277, 281, 286, 290, 294, 298, 302, 306, 310, 314, 318, 322, 326, 329, 333, 337, 341, 345, 348, 352, 356, 360, 106, 124, 137, 147, 155, 163, 171, 178, 184, 190, 196, 202, 208, 213, 218, 223, 228, 233, 238, 243, 247, 252, 256, 261, 265, 270, 274, 278, 282, 287, 291, 295, 299, 303, 307, 311, 315, 319, 323, 327, 331, 335, 338, 342, 346, 350, 354, 357, 106, 124, 137, 147, 156, 164, 171, 178, 185, 191, 197, 203, 208, 214, 219, 224, 229, 234, 239, 244, 248, 253, 258, 262, 266, 271, 275, 279, 284, 288, 292, 296, 300, 304, 308, 312, 316, 320, 324, 328, 332, 336, 340, 344, 347, 351, 355, 106, 125, 137, 147, 156, 164, 172, 179, 185, 192, 198, 204, 209, 215, 220, 225, 230, 235, 240, 245, 249, 254, 259, 263, 267, 272, 276, 281, 285, 289, 293, 297, 301, 305, 310, 314, 318, 322, 325, 329, 333, 337, 341, 345, 349, 353, 107, 125, 138, 148, 157, 165, 172, 179, 186, 192, 198, 204, 210, 215, 221, 226, 231, 236, 241, 246, 250, 255, 260, 264, 269, 273, 277, 282, 286, 290, 294, 298, 303, 307, 311, 315, 319, 323, 327, 331, 335, 338, 342, 346, 350};

uint32_t  CPU_Frequency_Mhz;
uint32_t CPU_tick_ns;

//uninitalised pointers to SPI objects
SPIClass * vspi = NULL;
SPIClass * hspi = NULL;
int vspi_mux_channel = 0;
int hspi_mux_channel = 0;
bool dotstars_enabled [NBSTRIPS] = {true, true, true, true};
//uninitalised pointers to SPI objects
My_Adafruit_DotStar * dotstar0 = NULL;
My_Adafruit_DotStar * dotstar1 = NULL;

// Global parameters
#define USB_SERIAL_BAUD 9600

// Communication parameters
const char* ssid = "DeformableFlyingDisplay";
const char* password = "123456789";
IPAddress ip(192, 168, 137, 171);
IPAddress dns(192, 168, 137,  1);
IPAddress gateway(192, 168, 137,  1);
IPAddress subnet(255, 255, 255, 0);

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// JSON data buffer
StaticJsonDocument<8192> jsonDocument;
char buffer[8192];

volatile int current_height;
int target_height;
float duration_height;
int status_height;
int target_height_asked;
bool new_target_height = false;

volatile int current_spin;
int target_spin;
int status_spin;
volatile int target_spin_asked;
int target_spin_cmd_asked;


volatile int min_current_line = 0;
volatile int current_lines [NBSTRIPS] = {0, 0, 0, 0};
volatile int target_lines [NBSTRIPS] = {0, 0, 0, 0};
volatile int status_lines [NBSTRIPS] = {0, 0, 0, 0};
volatile int vertical_resolutions [NBSTRIPS] = {0, 0, 0, 0};
volatile int reset_pixel = RESETPIXEL;
volatile int initial_line = reset_pixel * PITCHPIXEL;
volatile int initial_vertical_resolution = initial_line/PITCHPIXEL - RESETPIXELOFFSET; 
volatile int max_vertical_resolution = initial_vertical_resolution;
volatile int horizontal_resolutions [NBSTRIPS] = {0, 0, 0, 0};
const int initial_horizontal_resolution = max_horizontal_resolutions[0];
volatile int max_horizontal_resolution = initial_horizontal_resolution;
// image transformations
volatile float display_image_anchor_x_factor = 0.5;
volatile float display_image_anchor_y_factor = 0.5;
volatile float display_image_crop_top_factor = 0.0;
volatile float display_image_crop_right_factor = 0.0;
volatile float display_image_crop_bottom_factor = 0.0;
volatile float display_image_crop_left_factor = 0.0;
volatile float display_image_offset_x_factor = 0.0;
volatile float display_image_offset_y_factor = 0.0;

volatile float display_image_anchor_x_factor_bis = 0.5;
volatile float display_image_anchor_y_factor_bis = 0.5;
volatile float display_image_crop_top_factor_bis = 0.0;
volatile float display_image_crop_right_factor_bis = 0.0;
volatile float display_image_crop_bottom_factor_bis = 0.0;
volatile float display_image_crop_left_factor_bis = 0.0;
volatile float display_image_offset_x_factor_bis = 0.0;
volatile float display_image_offset_y_factor_bis = 0.0;

volatile int first_display_column_correction = 0;
volatile int second_display_column_correction = 0;





volatile int brightness = MAXBRIGHTNESS;

float duration_line;
int target_line_asked;
int index_line;
bool new_target_line = false;

float duration_lines;
int target_color = 0;
int target_lines_nb = 0;
int target_lines_index [NBSTRIPS] = {0,1,2,3};
int target_lines_position [NBSTRIPS] = {0,0,0,0};
float target_lines_duration [NBSTRIPS] = {1,1,1,1};
int target_lines_asked;
bool new_target_lines = false;

int target_light_asked;
int target_light_color = 0;
bool new_target_light = false;

int target_lights_asked;
int target_lights_color = 0;
bool new_target_lights = false;

bool new_reset = false;


unsigned long curr_time;
unsigned long prev_display_time = 0;
unsigned long spinner_prev_keep_alive_time;
unsigned long spinner_prev_refresh_state_time;
volatile bool spinner_keep_alive;
bool spinner_command_enabled = false;


int light_index = 0;
const int step = 8;
const unsigned long LIGHT_DELAY = 1000;
unsigned long prev_light_time;

volatile int start_light_index = NUMPIXELS;
volatile int end_light_index = NUMPIXELS;
volatile bool displayIsReset = true;
volatile bool displayToReset = false;
volatile bool displayToTurnOff = false;

volatile bool dualMode = false;
volatile bool pitchMode = false;

volatile int calibrated_display_rpm = 200;
volatile int display_rpm = 200;
volatile int target_display_rpm_asked = display_rpm;
volatile bool new_display_rpm = false;
QuickStats stats;



int total_line = 0;

volatile int src_half_image_width = 0;
volatile bool DisplayFeaturesToUpdate = false;
//first
volatile int start_image_column_index = 0;
volatile int start_image_row_index = 0;
volatile int display_column_index = 0;
volatile int display_column_offset = 0;
volatile int display_row_offset = 0;
volatile int offseted_display_column_index = 0;
volatile int prev_display_column_index = 0;
volatile int prev_first_display_column_index = 0;
volatile int display_column_indexes [NBSTRIPS] = {0,0,0,0};
volatile int prev_display_column_indexes [NBSTRIPS] = {0,0,0,0};

volatile int prev_second_display_column_index = 0;
volatile int error_display_column_index = 0;
volatile int diff_display_column_index = 0;
volatile int display_vertical_resolution = 0;
volatile int display_horizontal_resolution = 0;
volatile int display_horizontal_resolution_offset = 0;
volatile int horizontal_resolution = 0;
volatile int horizontal_resolution_offset = 0;
volatile int horizontal_resolution_asked;
volatile bool new_horizontal_resolution = false;
volatile int half_horizontal_resolution = 0;
volatile int half_horizontal_resolution_offset = 0;

// second
volatile int display_horizontal_resolution_bis = 0;
volatile int display_vertical_resolution_bis = 0;
volatile int display_horizontal_resolution_offset_bis = 0;
volatile int start_image_column_index_bis = 0;
volatile int start_image_row_index_bis = 0;
volatile int start_display_column_index_bis = 0;
volatile int end_display_column_index_bis = 0;
volatile int start_display_row_index_bis = 0;
volatile int end_display_row_index_bis = 0;
volatile int display_column_offset_bis = 0;
volatile  int display_row_offset_bis = 0;


volatile unsigned long display_rotation_delay = 0;
volatile unsigned long prev_display_rotation_time = 0;

int start_display_column_index = 0;
int end_display_column_index = DISPLAY_MAX_HORIZONTAL_RESOLUTION;
int start_display_row_index = 0;
int end_display_row_index = DISPLAY_MAX_VERTICAL_RESOLUTION;

volatile unsigned long target_time_spacing_ns = 0;
volatile unsigned long start_time_spacing_ns = 0;
volatile unsigned long end_time_spacing_ns = 0;
volatile unsigned long delta_time_spacing_ns = 0;
volatile unsigned long target_delta_time_spacing_ns = 0;
float prev_pixel_time = 0; 
unsigned curr_ccount = 0;
unsigned prev_ccount = 0;
//unsigned long curr_long_ccount = 0;
//unsigned long prev_long_ccount = 0;
unsigned long curr_nanos = 0;
unsigned long prev_nanos = 0;

int image_row_index = 0;
int image_column_index = 0;
int error_image_column_index = 0;
int prev_image_column_index = 0;
volatile int missed_bands_per_frame = 0;
unsigned int red = 0;
unsigned int green = 0;
unsigned int blue = 0;
byte color_8bits = 0;
unsigned int color_24bits = 0;
TaskHandle_t communicationTaskHandler;
volatile bool displayActive = false;
volatile bool debug = false;

volatile uint32_t src_image_width = DISPLAY_MAX_HORIZONTAL_RESOLUTION;
volatile uint32_t src_image_height = DISPLAY_MAX_VERTICAL_RESOLUTION;
volatile uint8_t src_image_pixels [DISPLAY_MAX_NB_PIXELS];

//int src_image_depth = 1; //bytes
//int src_image_nb_bytes = src_image_width * src_image_height * src_image_depth;//


//String socketData = "";
bool corrupted_frame = false;

// Calibration

volatile int index_strip = 0;
volatile unsigned long strip_off_start_time = 0; //ms
volatile unsigned long strip_on_start_time = 0; //ms
const unsigned long STRIP_OFF_DURATION = 500; //ms
const unsigned long STRIP_MONOCOLOR_DURATION = 1000;
const unsigned long STRIP_DUALCOLOR_DURATION = 1000;
const unsigned long STRIP_DUALCOLOR_PITCH_DURATION = 1000;
volatile int index_angular_speed = 0;
volatile bool angular_speed_is_triggered = false;
volatile bool angular_speed_is_reached = false;

const uint32_t NB_CALIB_ANGULAR_SPEED = 8;
const uint32_t NB_CALIB_DISPLAY_ERROR = NB_CALIB_ANGULAR_SPEED * NBSTRIPS;


int16_t calib_angular_speeds[NB_CALIB_ANGULAR_SPEED] = {150, 180, 210, 240, 270, 300, 330, 350}; //  {150, 180, 210}; // //{150, 165, 180, 195, 210, 225, 240, 255, 270, 285, 300, 315, 330, 345, 360};

uint32_t calib_target_deltatime_spacing[NB_CALIB_DISPLAY_ERROR]; 
uint32_t calib_deltatime_spacing_with_pitch[NB_CALIB_DISPLAY_ERROR];
uint32_t calib_deltatime_spacing_without_pitch[NB_CALIB_DISPLAY_ERROR];

int16_t calib_display_errors_with_pitch[NB_CALIB_DISPLAY_ERROR];
int16_t calib_display_errors_without_pitch[NB_CALIB_DISPLAY_ERROR];

int16_t calib_height[NB_CALIB_DISPLAY_ERROR];
int16_t calib_length[NB_CALIB_DISPLAY_ERROR];
int16_t calib_vertical_resolution[NB_CALIB_DISPLAY_ERROR];
int16_t calib_horizontal_resolution[NB_CALIB_DISPLAY_ERROR];

volatile bool calib_to_publish = false;


//10fps 12fps 14fps 16fps 18fps 20fps 22fps 24fps 
//{150, 180, 210, 240, 270, 300, 330, 360};

volatile bool calib_enabled = false;
volatile bool strips_done = false;
volatile int strip_error = 0;
volatile bool display_done = false;
volatile bool new_start_calibration = false;
volatile bool new_stop_calibration = false;
volatile bool strip_is_on = false;
volatile bool asked_to_count_error = false;
volatile bool counting_error = false;

uint32_t triangularIndex (float height, float length){
    uint32_t index = 0;
    uint32_t index_height = uint32_t(constrain(round((height - MIN_H)/STEP_H), 0, NB_H-1));
    uint32_t index_length = uint32_t(constrain(round((length - MIN_L)/STEP_L), 0, NB_L-1));
    uint32_t index_sum = 0;
    for (uint32_t i = 0; i < index_height; i++) index_sum += NB_L - i;
    index = index_sum + index_length + index_height;
    return index;
}

void delayNs(uint32_t nanoseconds){
  uint32_t currCycleCount = XTHAL_GET_CCOUNT();
  uint32_t prevCycleCount = currCycleCount;
  while((currCycleCount - prevCycleCount) * CPU_tick_ns < nanoseconds){
    currCycleCount = XTHAL_GET_CCOUNT();
  }
}

/*void selectMux(int mux_pin, int mux_channel){
  int A = bitRead(mux_channel, 0);
  digitalWrite(mux_pin, A);
}*/

void selectVspiMux(int mux_channel){ // 400-600 ns
  //unsigned long prev_nano_time = getNanos(30000000);
  vspi_mux_channel = mux_channel;
  int A = bitRead(vspi_mux_channel, 0);
  digitalWrite(VSPI_MUX, A);
  //unsigned long curr_nano_time = getNanos(30000000);
  // Serial.print("selectVspiMux duration (ns):");
  // Serial.println(curr_nano_time - prev_nano_time);
}

void selectHspiMux(int mux_channel){// 400-600ns
  //unsigned long prev_nano_time = getNanos(30000000);
  hspi_mux_channel = mux_channel;
  int A = bitRead(hspi_mux_channel, 0);
  digitalWrite(HSPI_MUX, A);
  //unsigned long curr_nano_time = getNanos(30000000);
  //Serial.print("selectHspiMux duration (ns):");
  //Serial.println(curr_nano_time - prev_nano_time);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo * info = (AwsFrameInfo*)arg;
  // fetch frame
  if(info->final && info->opcode == WS_BINARY){
    if (info -> index == 0) corrupted_frame = false;
    if(!corrupted_frame && info->index + len <= DISPLAY_MAX_NB_PIXELS){
      for (size_t i = 0; i < len; i++){
        src_image_pixels[info->index + i] = data[i];
      }
      if ( info->index + len == info->len){
        src_image_width =  ((uint32_t)src_image_pixels[info->len-2]) << 8 | (uint32_t)src_image_pixels[info->len-1];
        src_image_height =  ((uint32_t)src_image_pixels[info->len-4]) << 8 | (uint32_t)src_image_pixels[info->len-3];
        Serial.printf("Received frame: [%d x %d]\n",src_image_width, src_image_height);
      };

    } else {
       if(!corrupted_frame){
          Serial.printf("Corrupted frame detected! Skipping it...\n");
          corrupted_frame = true;
       }
    }
  } 

  // AwsFrameInfo *info = (AwsFrameInfo*)arg;

  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    deserializeJson(jsonDocument, ((char*)data));
    //serializeJsonPretty(jsonDocument, Serial);
    String topic = jsonDocument["topic"];
    //Serial.printf("Received message: [%s] %s\n", jsonDocument["topic"], jsonDocument["payload"]);

    if (topic == STATE_CMD){
      // jsonDocument.clear();
      // jsonDocument[HEIGHT_STATUS] = status_height;
      // jsonDocument[HEIGHT_CURRENT] = current_height;
      // jsonDocument[HEIGHT_TARGET] = target_height;
      // jsonDocument[SPIN_STATUS] = status_spin;
      // jsonDocument[SPIN_CURRENT] = current_spin;
      // jsonDocument[SPIN_TARGET] = target_spin;
      // serializeJson(jsonDocument, buffer);
      ws.textAll((char*)buffer);
    }
    else if (topic == OFFSET_CMD){
      display_image_offset_x_factor  = jsonDocument["payload"][X_CMD];
      display_image_offset_y_factor = jsonDocument["payload"][Y_CMD];
      Serial.printf("Received command: offset %f %f\n", display_image_offset_x_factor, display_image_offset_y_factor);
    }
    else if (topic == OFFSET_BIS_CMD){
      display_image_offset_x_factor_bis  = jsonDocument["payload"][X_CMD];
      display_image_offset_y_factor_bis = jsonDocument["payload"][Y_CMD];
      Serial.printf("Received command: offset_bis %f %f\n", display_image_offset_x_factor_bis, display_image_offset_y_factor_bis);
    }
  else if (topic == COLUMN_CORRECTION_CMD){
      first_display_column_correction = jsonDocument["payload"][FIRST_CMD];
      second_display_column_correction = jsonDocument["payload"][SECOND_CMD];
      Serial.printf("Received command: column_correction %d %d\n", first_display_column_correction, second_display_column_correction);
    }

    else if (topic == ROTATION_CMD){
      int rotation_percent_per_second = jsonDocument["payload"][ROTATION_CMD];
      if (rotation_percent_per_second > 0){
        display_rotation_delay = 1000/rotation_percent_per_second;
      } else {
        display_rotation_delay = 0;
      }
      Serial.printf("Received command: rotation %d\n", display_rotation_delay);
    }

    else if (topic == ANCHOR_CMD){
      display_image_anchor_x_factor  = jsonDocument["payload"][X_CMD];
      display_image_anchor_y_factor = jsonDocument["payload"][Y_CMD];
      Serial.printf("Received command: anchor %f %f\n", display_image_anchor_x_factor, display_image_anchor_y_factor);
    }
    else if (topic == ANCHOR_BIS_CMD){
      display_image_anchor_x_factor_bis  = jsonDocument["payload"][X_CMD];
      display_image_anchor_y_factor_bis = jsonDocument["payload"][Y_CMD];
      Serial.printf("Received command: anchor_bis %f %f\n", display_image_anchor_x_factor_bis, display_image_anchor_y_factor_bis);
    }
    else if (topic == CROP_CMD){
      display_image_crop_top_factor  = jsonDocument["payload"][TOP_CMD];
      display_image_crop_right_factor = jsonDocument["payload"][RIGHT_CMD];
      display_image_crop_bottom_factor = jsonDocument["payload"][BOTTOM_CMD];
      display_image_crop_left_factor = jsonDocument["payload"][LEFT_CMD];
      Serial.printf("Received command: crop %f %f %f %f\n", display_image_crop_top_factor, display_image_crop_right_factor, display_image_crop_bottom_factor, display_image_crop_left_factor);
    }
    else if (topic == CROP_BIS_CMD){
      display_image_crop_top_factor_bis  = jsonDocument["payload"][TOP_CMD];
      display_image_crop_right_factor_bis = jsonDocument["payload"][RIGHT_CMD];
      display_image_crop_bottom_factor_bis = jsonDocument["payload"][BOTTOM_CMD];
      display_image_crop_left_factor_bis = jsonDocument["payload"][LEFT_CMD];
      Serial.printf("Received command: crop_bis %f %f %f %f\n", display_image_crop_top_factor_bis, display_image_crop_right_factor_bis, display_image_crop_bottom_factor_bis, display_image_crop_left_factor_bis);
    }
    else if (topic == START_CALIBRATION_CMD){
      Serial.println("Received command: start_calibration");
      new_start_calibration = true;
    }
    else if (topic == STOP_CALIBRATION_CMD){
      Serial.println("Received command: stop_calibration");
      new_stop_calibration = true;
    }
    else if (topic == DUAL_ENABLES_CMD){
      Serial.println("Received command: dual_enables");
      dualMode = true;
    }
    
    else if (topic == PITCH_ENABLE_CMD){
      Serial.println("Received command: pitch_enable");
      pitchMode = true;
    }
    else if (topic == ENABLES_CMD){
      Serial.println("Received command: enables");
      displayActive = true;
    }
    else if (topic == ENABLE_CMD){
      int index = jsonDocument["payload"][ENABLE_CMD];
      Serial.printf("Received command: enable %d\n", index);
      dotstars_enabled[index] = true;
    }
    

    else if (topic == DUAL_DISABLES_CMD){
      Serial.println("Received command: dual_disables");
      dualMode = false;
    }
    
    else if (topic == DISABLES_CMD){
      Serial.println("Received command: disables");
      displayActive = false;
      displayToTurnOff = true;
    }
    
    else if (topic == PITCH_DISABLE_CMD){
      Serial.println("Received command: pitch_disable");
      pitchMode = false;
    }

    else if (topic == DISABLE_CMD){
      int index = jsonDocument["payload"][DISABLE_CMD];
      Serial.printf("Received command: disable %d\n", index);
      dotstars_enabled[index] = false;
    }

    else if (topic == BRIGHTNESS_CMD){
      brightness = jsonDocument["payload"][BRIGHTNESS_CMD];
      Serial.printf("Received command: brightness %d\n", brightness);
    }

    else if (topic == DISPLAY_RPM_CMD){
      target_display_rpm_asked = jsonDocument["payload"][DISPLAY_RPM_CMD];
      Serial.printf("Received command: display_rpm %d\n", target_display_rpm_asked);
      new_display_rpm = true;
    }
    // else if (topic == HORIZONTAL_RESOLUTION_CMD){
    //   horizontal_resolution_asked = jsonDocument["payload"][HORIZONTAL_RESOLUTION_CMD];
    //   Serial.printf("Received command: horizontal_resolution %d\n", horizontal_resolution_asked);
    //   new_horizontal_resolution = true;
    // }


    else if (topic == SPIN_CMD){
      target_spin_asked = jsonDocument["payload"][SPIN_CMD];
      Serial.printf("Received command: spin %d\n", target_spin_asked);
      spinner_command_enabled = false;
      spinner_keep_alive = true;
    }
    else if (topic == SPIN_CMD_CMD){
      target_spin_cmd_asked = jsonDocument["payload"][SPIN_CMD_CMD];
      Serial.printf("Received command: spinCmd %d\n", target_spin_cmd_asked);
      spinner_command_enabled = true;
      spinner_keep_alive = true;
    }
    else if (topic == RESET_CMD){
      int reset_pixel_offset = jsonDocument["payload"][RESET_CMD];
      reset_pixel = RESETPIXEL + reset_pixel_offset;
      initial_line = reset_pixel * PITCHPIXEL;
      initial_vertical_resolution = initial_line/PITCHPIXEL - RESETPIXELOFFSET; 
      Serial.printf("Received command: reset %d\n", reset_pixel_offset);
      new_reset = true;
    }

    else if (topic == HEIGHT_CMD){
      target_height_asked = jsonDocument["payload"][HEIGHT_CMD];
      duration_height = jsonDocument["payload"][DURATION_CMD];
      Serial.printf("Received command: height %d %f\n", target_height_asked, duration_height);
      new_target_height = true;
    }
    
    
    else if (topic == LINE_CMD){
      target_line_asked = jsonDocument["payload"][LINE_CMD];
      duration_line = jsonDocument["payload"][DURATION_CMD];
      index_line = jsonDocument["payload"][INDEX_CMD];
      Serial.printf("Received command: line %d %d %f\n", index_line, target_line_asked, duration_line);
      new_target_line = true;
    }

    else if (topic == LINES_CMD){
      int indexSize =  jsonDocument["payload"][LINES_INDEX_CMD].size();
      int positionSize =  jsonDocument["payload"][LINES_POSITION_CMD].size();
      int durationSize =  jsonDocument["payload"][LINES_DURATION_CMD].size();
      if (indexSize == positionSize && positionSize ==  durationSize && durationSize == indexSize && indexSize <= NBSTRIPS){
        target_lines_nb = indexSize;
        for (int i = 0; i < indexSize; i++){
          target_lines_index[i] = jsonDocument["payload"][LINES_INDEX_CMD][i];
          target_lines_position[i] = jsonDocument["payload"][LINES_POSITION_CMD][i];
          target_lines_duration[i] = jsonDocument["payload"][LINES_DURATION_CMD][i];
          Serial.printf("Received command: lines %s %s %s\n",  jsonDocument["payload"][LINES_INDEX_CMD].as<String>(), jsonDocument["payload"][LINES_POSITION_CMD].as<String>(), jsonDocument["payload"][LINES_DURATION_CMD].as<String>());
        }
      }
      new_target_lines = true;
    }


    else if (topic == LIGHT_CMD){
      target_light_asked = jsonDocument["payload"][LIGHT_CMD];
      target_light_color = jsonDocument["payload"][COLOR_CMD];
      Serial.printf("Received command: light %d %06X\n", target_light_asked, target_light_color);
      new_target_light = true;
    }

    else if (topic == LIGHTS_CMD){
      target_lights_asked = jsonDocument["payload"][LIGHTS_CMD];
      target_lights_color = jsonDocument["payload"][COLOR_CMD];
      Serial.printf("Received command: lights %d %06X\n", target_lights_asked, target_light_color);
      new_target_lights = true;
    }
        
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
      Serial.printf("WS_EVT_PONG\n");
      break;
    case WS_EVT_ERROR:
      Serial.printf("WS_EVT_ERROR\n");
      break;
  }
}

void setup(void) {
  spinner_keep_alive = false;
  curr_time = spinner_prev_keep_alive_time = millis();
  
  CPU_Frequency_Mhz = getCpuFrequencyMhz();
  CPU_tick_ns = 1.0 / (CPU_Frequency_Mhz / 1000.0);
  // setup serial communication for debugging
  Serial.begin(SERIAL_BAUDRATE);
  Serial2.begin(SERIAL_BAUDRATE, SERIAL_8N1, RXD2, TXD2);
  // start light
  
  vspi = new SPIClass(VSPI);
  hspi = new SPIClass(HSPI);
  vspi->begin();
  hspi->begin();
  dotstar0 = new My_Adafruit_DotStar(NUMPIXELS,  DOTSTAR_BGR, vspi, SPI_FREQUENCY); 
  dotstar1 = new My_Adafruit_DotStar(NUMPIXELS, DOTSTAR_BGR, hspi, SPI_FREQUENCY);
  pinMode(VSPI_MUX, OUTPUT); 
  pinMode(HSPI_MUX, OUTPUT); 
  //turn off strip 0 and 1
  dotstar0 -> clear();
  dotstar1 -> clear();
  selectVspiMux(0);
  selectHspiMux(0);
  delayNs(MUX_NANOSECONDS_DELAY);
  dotstar0 -> show(); 
  dotstar1 -> show(); 
  // turn off strip 2 and 3
  dotstar0 -> clear();
  dotstar1 -> clear();
  selectVspiMux(1);
  selectHspiMux(1);
  delayNs(MUX_NANOSECONDS_DELAY);
  dotstar0 -> show(); 
  dotstar1 -> show(); 


  // setup the WiFi network
  startWifiWebServer();
  new_target_height = false;
  
  // ESP_LOGE("Memory", "heap_caps_get_free_size: %d", heap_caps_get_free_size(MALLOC_CAP_8BIT));
  // ESP_LOGE("Memory", "heap_caps_get_minimum_free_size: %d", heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));
  // ESP_LOGE("Memory", "heap_caps_get_largest_free_block: %d", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  // ESP_LOGE("Memory", "image_required_size: %d", src_image_nb_bytes);
  // src_image_pixels = (byte*)calloc(src_image_nb_bytes, sizeof(byte)); // src_image_width * src_image_height  * src_image_pixel_size
  // if(src_image_pixels == NULL){
  //   Serial.println("Failed to allocate memory for image."); 
  // } else {
  //   free(src_image_pixels);
  //   //Serial.println("OK!"); 
  // }

}

void startWifiWebServer(){
  // setup the WiFi network
  WiFi.mode(WIFI_STA);
  WiFi.config(ip, gateway, subnet, dns);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP()); 

  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.begin();
  BaseType_t ans = xTaskCreate(&communicationTask, "communicationTask", 4096, NULL, 1, &communicationTaskHandler);
    if (ans == pdPASS){
    } else {
      ESP_LOGE("communicationTask", "Failed to generate a communicationTask");
    }

}



// void makeColorGradient(float frequency1, float frequency2, float frequency3, float phase1, float phase2, float phase3, float center, float width, float len){
//     if ( millis() - prev_light_time > LIGHT_DELAY){
//       light_index = light_index + 1;
//       if (light_index > len - 1) light_index = 0;
//       red = sin(frequency1 * light_index + phase1) * width + center;
//       green = sin(frequency2 * light_index + phase2) * width + center;
//       blue = sin(frequency3 * light_index + phase3) * width + center; 
//       color  = red << 16 | green << 8 | blue;
//       for (int i = 0; i < NUMPIXELS; i++){
//         dotstar0 -> setPixelColor(i, color); // 'On' pixel at head
//       }
//       dotstar0 -> show(); 
//       prev_light_time = millis();
//     }
// }


void communicationTask(void *pvParameter) {

  while(true){
    curr_time = millis();
    handleServer();
    handleState();
    handleSerial();
    vTaskDelay(10);
  }
  
}

void handleServer(){
  // CHECK WIFI CONNECTION
    if (WiFi.status() != WL_CONNECTED) {
          Serial.println("Wifi connection lost... stopping everything.");
          //Serial.println(WiFi.localIP()); 
          spinner_keep_alive = false;
          startWifiWebServer();
    } else {
      // CHECK FOR WEBSOCKET CLIENT
      ws.cleanupClients();
    }
}

void handleSerial(){

  // FORWARD DEBUG COMMANDS
  if(Serial.available() > 0){
      // Serial.print("receiving from Serial (sending to Serial2): ");
      // String str = Serial.readStringUntil('\n');
      // Serial.println(str);
      // Serial2.println(str);
      String str = Serial.readStringUntil('\n');
      handleDebugCmd(str, Serial);
      String resetCmd = "reset ";
      int resetIndex = str.indexOf(resetCmd);
      if(resetIndex != -1){
          displayIsReset = false;
          displayToReset = true;
      }


      String lightCmd = "light ";
       int lightIndex = str.indexOf(lightCmd);
       if(lightIndex != -1){
            str = str.substring(lightIndex + lightCmd.length());
            start_light_index = NUMPIXELS - str.toInt();
            end_light_index = NUMPIXELS - str.toInt() + 1;
       }
      String lightsCmd = "lights ";
        int lightsIndex = str.indexOf(lightsCmd);
        if(lightsIndex != -1){
            str = str.substring(lightsIndex + lightsCmd.length());
            start_light_index = NUMPIXELS - str.toInt();
            end_light_index = NUMPIXELS;
        }
  }
  // FETCH STATUS
  if(Serial2.available() > 0){
      String str = Serial2.readStringUntil('\n');
      //Serial.println(str);
      DeserializationError err = deserializeJson(jsonDocument, str);
       if (err == DeserializationError::Ok){
        String topic = jsonDocument[TOPIC];
        if (topic == STATE_CMD){
            if (calib_to_publish){
              jsonDocument.clear(); 
              JsonObject calibrationDocument = jsonDocument.createNestedObject(EXPERIMENTATION);

              JsonArray calib_angular_speeds_array = calibrationDocument.createNestedArray(ANGULAR_SPEED);

              JsonArray calib_horizontal_resolution_array = calibrationDocument.createNestedArray(HORIZONTAL_RESOLUTION);
              JsonArray calib_vertical_resolution_array = calibrationDocument.createNestedArray(VERTICAL_RESOLUTION);

              JsonArray calib_height_array = calibrationDocument.createNestedArray(HEIGHT);
              JsonArray calib_length_array = calibrationDocument.createNestedArray(LENGTH);

              JsonArray calib_target_deltatime_spacing_array = calibrationDocument.createNestedArray(TARGET_DELTATIME_SPACING);

              JsonArray calib_display_errors_with_pitch_array = calibrationDocument.createNestedArray(ERROR_WITH_PITCH);
              JsonArray calib_deltatime_spacing_with_pitch_array = calibrationDocument.createNestedArray(DELTATIME_SPACING_WITH_PITCH);
              JsonArray calib_display_errors_without_pitch_array = calibrationDocument.createNestedArray(ERROR_WITHOUT_PITCH);
              JsonArray calib_deltatime_spacing_without_pitch_array = calibrationDocument.createNestedArray(DELTATIME_SPACING_WITHOUT_PITCH);

              for(int i = 0; i < NB_CALIB_ANGULAR_SPEED; i++){
                calib_angular_speeds_array.add(calib_angular_speeds[i]);
              }

              for(int i = 0; i < NB_CALIB_DISPLAY_ERROR; i++){

                calib_horizontal_resolution_array.add(calib_horizontal_resolution[i]);
                calib_vertical_resolution_array.add(calib_vertical_resolution[i]);

                calib_height_array.add(calib_height[i]);
                calib_length_array.add(calib_length[i]);

                calib_target_deltatime_spacing_array.add(calib_target_deltatime_spacing[i]);

                calib_display_errors_with_pitch_array.add(calib_display_errors_with_pitch[i]);
                calib_deltatime_spacing_with_pitch_array.add(calib_deltatime_spacing_with_pitch[i]);

                calib_display_errors_without_pitch_array.add(calib_display_errors_without_pitch[i]);
                calib_deltatime_spacing_without_pitch_array.add(calib_deltatime_spacing_without_pitch[i]);
              }
              calib_to_publish = false;
          } else {
            current_height = jsonDocument[PAYLOAD][HEIGHT_CURRENT];
            target_height = jsonDocument[PAYLOAD][HEIGHT_TARGET];
            status_height = jsonDocument[PAYLOAD][HEIGHT_STATUS];
            current_spin = jsonDocument[PAYLOAD][SPIN_CURRENT];
            display_rpm = (current_spin > 1) ? RPM_TO_FPS_ERROR * current_spin : 200;
            target_spin = jsonDocument[PAYLOAD][SPIN_TARGET];
            status_spin = jsonDocument[PAYLOAD][SPIN_STATUS];
            JsonArray line_status_array = jsonDocument[PAYLOAD][LINE_STATUS].as<JsonArray>();
            JsonArray line_current_array = jsonDocument[PAYLOAD][LINE_CURRENT].as<JsonArray>();
            JsonArray line_target_array = jsonDocument[PAYLOAD][LINE_TARGET].as<JsonArray>();
            min_current_line = current_lines[0];
            max_vertical_resolution = initial_vertical_resolution;
            max_horizontal_resolution = initial_horizontal_resolution;
            for(int i = 0; i < NBSTRIPS; i++){
              status_lines[i] = line_status_array[i];
              current_lines[i] = line_current_array[i];
              target_lines[i] = line_target_array[i];
              vertical_resolutions[i] = (initial_line + current_lines[i])/PITCHPIXEL - RESETPIXELOFFSET;
              if (vertical_resolutions[i] > max_vertical_resolution) max_vertical_resolution = vertical_resolutions[i];
              horizontal_resolutions[i] = max_horizontal_resolutions[triangularIndex(MIN_H + current_height, initial_line  + current_lines[i])]; //TODO
              if (horizontal_resolutions[i] > max_horizontal_resolution) max_horizontal_resolution =  horizontal_resolutions[i];
              // Serial.print("line ");
              // Serial.print(i);
              // Serial.print(" ver ");
              // Serial.print(vertical_resolutions[i] );
              // Serial.print(" hor ");
              // Serial.print(horizontal_resolutions[i] );
              // Serial.println();
              if (current_lines[i] < min_current_line) min_current_line = current_lines[i];
            } 
            // update display error
            jsonDocument[PAYLOAD][DISPLAY_VERTICAL_RESOLUTION] = max_vertical_resolution;
            jsonDocument[PAYLOAD][DISPLAY_HORIZONTAL_RESOLUTION] = max_horizontal_resolution;
            jsonDocument[PAYLOAD][DISPLAY_ERROR] = missed_bands_per_frame;
          }
          serializeJson(jsonDocument, buffer);
          ws.textAll((char*)buffer);
          DisplayFeaturesToUpdate = true;
        }
        if (topic == RESET_CMD){
          Serial.println("Received response: reset");
          displayIsReset = true;
        }
       } else {
         Serial.print(err.c_str());
         Serial.print(" ");
         Serial.println(str);
       }
  }
}

void handleDebugCmd(String message, Stream &serial){
    String str =  message.substring(0);
    String debugCmd = DEBUG_CMD;
    int debugIndex = str.indexOf(debugCmd);
    if(debugIndex != -1){
        Serial.println("Received response: debug");
        str = str.substring(debugIndex + debugCmd.length());
        debug = str.toInt();
    }
}

void handleState(){


  // REPEATEDLY REQUEST VELOCITY TO KEEP SPINNER ALIVE
  if(spinner_keep_alive && curr_time - spinner_prev_keep_alive_time > SPINNER_KEEP_ALIVE_DELAY){
    if (spinner_command_enabled){
      Serial2.print(SPIN_CMD_CMD);
      Serial2.print(" ");
      Serial2.println(target_spin_cmd_asked);
    } else {
      Serial2.print(SPIN_CMD);
      Serial2.print(" ");
      Serial2.println(target_spin_asked);
    }
    spinner_prev_keep_alive_time = curr_time;
  }

  // RESET REQUEST
  if (new_reset){
    displayIsReset = false;
    displayToReset = true;
    displayToTurnOff = true;
    new_reset = false;
  }

  if (DisplayFeaturesToUpdate){
    updateDisplaysFeatures();
    DisplayFeaturesToUpdate = false;
  }

  // HEIGHT REQUEST
  if(new_target_height){
    Serial2.print(HEIGHT_CMD);
    Serial2.print(" ");
    Serial2.print(target_height_asked);
    Serial2.print(" ");
    Serial2.println(duration_height);
    new_target_height = false;
  }

  // LINE REQUEST
  if(new_target_line){
    Serial2.print(LINE_CMD);
    Serial2.print(" ");
    Serial2.print(index_line);
    Serial2.print(" ");
    Serial2.print(target_line_asked);
    Serial2.print(" ");
    Serial2.println(duration_line);
    new_target_line = false;
  }

  // LINEs REQUEST
  if(new_target_lines){
    for (int i = 0; i < target_lines_nb; i++){
      Serial2.print(LINE_CMD);
      Serial2.print(" ");
      Serial2.print(target_lines_index[i]);
      Serial2.print(" ");
      Serial2.print(target_lines_position[i]);
      Serial2.print(" ");
      Serial2.println(target_lines_duration[i]);
    }
    new_target_lines = false;
  }

  // RESET REQUEST
  if (new_display_rpm){
    display_rpm = target_display_rpm_asked;
    new_display_rpm = false;
  }

  if (new_horizontal_resolution){
    max_horizontal_resolution = horizontal_resolution_asked;
    new_horizontal_resolution = false;
  }

  // LIGHT REQUEST
  if (new_target_light){
      start_light_index = NUMPIXELS - target_light_asked;
      end_light_index = NUMPIXELS - target_light_asked + 1;
      target_color = target_light_color;
      new_target_light = false;
  }
  // LIGHTS REQUEST
  if (new_target_lights){
      start_light_index = NUMPIXELS - target_lights_asked;
      end_light_index = NUMPIXELS;
      target_color = target_lights_color;
      new_target_lights = false;
  }

  if (new_start_calibration){
      Serial.println("new_start_calibration");
      startCalibration();
      Serial.println("setting new_start_calibration to false...");
      new_start_calibration = false;
      Serial.println("new_start_calibration is set to false!");
  }
  if (new_stop_calibration){
      Serial.println("new_stop_calibration");
      stopCalibration();
      new_stop_calibration = false;
  }
  // REPEATEDLY REQUEST STATE
  if(curr_time - spinner_prev_refresh_state_time > SPINNER_REFRESH_STATE_DELAY){
    
    //Serial.printf("CPU Frequency: %u, target delta time spacing: %u, delta time spacing: %u, missed_bands_per_frame: %d \n", CPU_Frequency_Mhz, target_delta_time_spacing_ns, delta_time_spacing_ns, missed_bands_per_frame);
    Serial2.println(STATE_CMD);
    spinner_prev_refresh_state_time = curr_time;
    
  }
}

void setStripFromFrame(My_Adafruit_DotStar * dotstar, int start_pixel_index, int end_pixel_index, int start_crop_index, int end_crop_index, int img_column_index, int img_row_index, int img_row_offset){
    
    for (int i = NUMPIXELS; i >= 0; i--){
        if(i >= start_pixel_index + img_row_offset && i < end_pixel_index + img_row_offset){
          int j = ((end_pixel_index + img_row_offset - 1) - i); // index between 0 and vertical_resolution
          if (j > start_crop_index && j < end_crop_index) { // cropping
            j = j + img_row_index;
            int pixel_index = src_image_width * j + img_column_index; 
            if (pixel_index < src_image_height * src_image_width){
                color_8bits =  src_image_pixels[pixel_index];
                red   = (color_8bits >> 5) * brightness / 7;
                green = ((color_8bits >> 2) & 0x07) * brightness / 7;
                blue  = (color_8bits & 0x03) * brightness / 3;
                color_24bits  = red << 16 | green << 8 | blue;
                dotstar -> setPixelColor(i, color_24bits);
            } else {
              dotstar -> setPixelColor(i, 0);
            }
          } else{
            dotstar -> setPixelColor(i, 0);
          }
          
        } else {
          dotstar -> setPixelColor(i, 0);
        }
    }
}

void setStripFromColor(My_Adafruit_DotStar * dotstar, int start_pixel_index, int end_pixel_index, int color){
    for (int i = end_pixel_index-1; i >= start_pixel_index; i--){
        int j = (end_pixel_index - 1) - i;
        dotstar -> setPixelColor(i, color);
    }
}

unsigned long getNanos(unsigned long modulo){
        // handle nanosecond counter
        unsigned curr_ccount = XTHAL_GET_CCOUNT();
        unsigned delta_ccount;
        if(curr_ccount > prev_ccount){
            delta_ccount = curr_ccount - prev_ccount;
        } else {
            delta_ccount = curr_ccount + (65535 - prev_ccount);
        }
        prev_ccount = curr_ccount;
        // add to long nanosecond counter
        //curr_long_ccount += delta_ccount;
        curr_nanos = (curr_nanos + delta_ccount * CPU_tick_ns) % modulo;
        // if(curr_nanos < prev_nanos){
        //     Serial.print(curr_nanos);
        //     Serial.print(" < ");
        //     Serial.print(prev_nanos);
        //     curr_nanos = curr_nanos + (4294967295 - prev_nanos);
        //     Serial.print(" -> ");
        //     Serial.print(curr_nanos);
        //     Serial.println();
        // }
        //prev_nanos = curr_nanos;
        return curr_nanos;

}

void persistenceOfVision(int target_display_rpm){
        target_time_spacing_ns = 1000/(target_display_rpm/60.0) * 1000000; //time for one turn in ms (2ms default) //(current_spin > 0) ? 1000/(current_spin/60.0) :  displayRPM
        display_horizontal_resolution = max_horizontal_resolution;
        display_vertical_resolution = max_vertical_resolution;
        display_horizontal_resolution_offset = display_horizontal_resolution/NBSTRIPS;
        target_delta_time_spacing_ns  = target_time_spacing_ns/display_horizontal_resolution;
        start_time_spacing_ns = getNanos(target_time_spacing_ns);
        display_column_index = start_time_spacing_ns/(float)target_time_spacing_ns  * display_horizontal_resolution;
        bool updating_first_strips = false;

        // first index of image
        start_image_column_index = display_image_anchor_x_factor * src_image_width - display_image_anchor_x_factor * display_horizontal_resolution;
        start_image_row_index = display_image_anchor_y_factor * src_image_height - display_image_anchor_y_factor * display_vertical_resolution;
        // crop of display
        start_display_column_index = display_image_crop_left_factor * display_horizontal_resolution;
        end_display_column_index = display_horizontal_resolution - display_image_crop_right_factor * display_horizontal_resolution;
        start_display_row_index = display_image_crop_top_factor * display_vertical_resolution;
        end_display_row_index = display_vertical_resolution - display_image_crop_bottom_factor * display_vertical_resolution;
        // offset of display
        display_column_offset = display_image_offset_x_factor * display_horizontal_resolution;
        display_row_offset = display_image_offset_y_factor * display_vertical_resolution;

        if (display_column_index != prev_first_display_column_index){
            selectVspiMux(0); // 400-600 ns
            selectHspiMux(1); // 400-600 ns
            updating_first_strips = true;
            
            offseted_display_column_index = display_column_index + display_column_offset;
            image_column_index  = start_image_column_index + offseted_display_column_index;
            if(
                dotstars_enabled[0] 
                && start_display_column_index <= offseted_display_column_index && offseted_display_column_index <= end_display_column_index 
                && image_column_index >= start_image_column_index &&  image_column_index < src_image_width 
              ){
              setStripFromFrame(dotstar0, NUMPIXELS0 - display_vertical_resolution, NUMPIXELS0, start_display_row_index, end_display_row_index, image_column_index, start_image_row_index, display_row_offset);
              
            }
            else dotstar0 -> clear();

            offseted_display_column_index =  (display_column_index + 2 * display_horizontal_resolution_offset) % display_horizontal_resolution + display_column_offset;
            image_column_index  = start_image_column_index + offseted_display_column_index;
            if(
                dotstars_enabled[2]
                && start_display_column_index <= offseted_display_column_index && offseted_display_column_index <= end_display_column_index 
                && image_column_index >= start_image_column_index && image_column_index < src_image_width
              ) {
                
              setStripFromFrame(dotstar1, NUMPIXELS2 - display_vertical_resolution, NUMPIXELS2, start_display_row_index, end_display_row_index, image_column_index, start_image_row_index, display_row_offset);
              
            }
            else dotstar1 -> clear();

            dotstar0 -> show(); // before 9200000 - 930000 ns CPU@240Mhz  SPI@20Mhz -> after 280000 - 297000 ns CPU@240Mhz  SPI@20Mhz ( 3 x FASTER!)
            //prev_nano_time = getNanos(30000000); 
            dotstar1 -> show();  // before 9200000 - 930000 ns CPU@240Mhz  SPI@20Mhz -> after 280000 - 297000 ns CPU@240Mhz  SPI@20Mhz ( 3 x FASTER!)
            //curr_nano_time = getNanos(30000000);
            //Serial.print(" dotstar1 -> show() (ns):");
            //Serial.println(curr_nano_time - prev_nano_time);
            if(pitchMode){
              dotstar0 -> clear();
              dotstar1 -> clear();
              dotstar0 -> show(); // before 9200000 - 930000 ns CPU@240Mhz  SPI@20Mhz -> after 280000 - 297000 ns CPU@240Mhz  SPI@20Mhz ( 3 x FASTER!)
              dotstar1 -> show(); // before 9200000 - 930000 ns CPU@240Mhz  SPI@20Mhz -> after 280000 - 297000 ns CPU@240Mhz  SPI@20Mhz ( 3 x FASTER!)
            }
            prev_first_display_column_index = display_column_index;
        } 
        
        prev_display_column_index = display_column_index;

        end_time_spacing_ns = getNanos(target_time_spacing_ns);
        display_column_index = end_time_spacing_ns/(float)target_time_spacing_ns  * display_horizontal_resolution;
        bool updating_second_strips = false;
        
        if (display_column_index != prev_second_display_column_index){
            updating_second_strips = true;
            selectVspiMux(1); // 400-600 ns
            selectHspiMux(0); // 400-600 ns
            
            offseted_display_column_index =  (display_column_index + display_horizontal_resolution_offset) % display_horizontal_resolution + display_column_offset;
            image_column_index  = start_image_column_index + offseted_display_column_index;
            if(
              dotstars_enabled[1]
              && start_display_column_index <= offseted_display_column_index && offseted_display_column_index <= end_display_column_index
              &&  image_column_index >= start_image_column_index && image_column_index < src_image_width
              ){

              setStripFromFrame(dotstar0, NUMPIXELS1 - display_vertical_resolution, NUMPIXELS1, start_display_row_index, end_display_row_index, image_column_index, start_image_row_index, display_row_offset);
              
            }
            else dotstar0 -> clear();

            offseted_display_column_index =   (display_column_index + 3 * display_horizontal_resolution_offset) % display_horizontal_resolution + display_column_offset;
            image_column_index  = start_image_column_index + offseted_display_column_index;
            if(
              dotstars_enabled[3]
              && start_display_column_index <= offseted_display_column_index && offseted_display_column_index <= end_display_column_index
              &&  image_column_index >= start_image_column_index && image_column_index < src_image_width){ 
                
              setStripFromFrame(dotstar1, NUMPIXELS3 - display_vertical_resolution, NUMPIXELS3, start_display_row_index, end_display_row_index, image_column_index, start_image_row_index, display_row_offset);
              
             // setStripFromFrame(dotstar1, NUMPIXELS3 - end_display_row_index, NUMPIXELS3 - start_display_row_index, image_column_index, start_image_row_index);
            }
            else dotstar1 -> clear();
            dotstar0 -> show(); // before 9200000 - 930000 ns CPU@240Mhz  SPI@20Mhz -> after 280000 - 297000 ns CPU@240Mhz  SPI@20Mhz ( 3 x FASTER!)
            dotstar1 -> show(); // before 9200000 - 930000 ns CPU@240Mhz  SPI@20Mhz -> after 280000 - 297000 ns CPU@240Mhz  SPI@20Mhz ( 3 x FASTER!)
        
            if(pitchMode){
              dotstar0 -> clear();
              dotstar1 -> clear();
              dotstar0 -> show(); // before 9200000 - 930000 ns CPU@240Mhz  SPI@20Mhz -> after 280000 - 297000 ns CPU@240Mhz  SPI@20Mhz ( 3 x FASTER!)
              dotstar1 -> show(); // before 9200000 - 930000 ns CPU@240Mhz  SPI@20Mhz -> after 280000 - 297000 ns CPU@240Mhz  SPI@20Mhz ( 3 x FASTER!)
            }
            prev_second_display_column_index = display_column_index;
        }

        prev_display_column_index = display_column_index;
        end_time_spacing_ns = getNanos(target_time_spacing_ns);
        if (updating_first_strips || updating_second_strips){
            delta_time_spacing_ns = end_time_spacing_ns - start_time_spacing_ns;
            if (end_time_spacing_ns < start_time_spacing_ns) {
              delta_time_spacing_ns =  end_time_spacing_ns + ( target_time_spacing_ns - start_time_spacing_ns);
            }
            if (!(updating_first_strips && updating_second_strips)) delta_time_spacing_ns = 2 * delta_time_spacing_ns;
            missed_bands_per_frame = max_horizontal_resolution - target_delta_time_spacing_ns/(float)delta_time_spacing_ns * max_horizontal_resolution;
        }

}

void updateDisplaysFeatures(){
        src_half_image_width = src_image_width/2;
        target_time_spacing_ns = 1000/(display_rpm/60.0) * 1000000; //time for one turn in ms (2ms default) //(current_spin > 0) ? 1000/(current_spin/60.0) :  displayRPM
        // first display
        display_horizontal_resolution = max(horizontal_resolutions[0], horizontal_resolutions[2]);
        display_horizontal_resolution = min(display_horizontal_resolution, src_half_image_width);
        display_vertical_resolution = max(vertical_resolutions[0], vertical_resolutions[2]);
        display_horizontal_resolution_offset = display_horizontal_resolution/NBSTRIPS;

        // first index of image
        start_image_column_index = display_image_anchor_x_factor * src_half_image_width - display_image_anchor_x_factor * display_horizontal_resolution;
        start_image_row_index = display_image_anchor_y_factor * src_image_height - display_image_anchor_y_factor * display_vertical_resolution;
        // crop of display
        start_display_column_index = display_image_crop_left_factor * display_horizontal_resolution;
        end_display_column_index = display_horizontal_resolution - display_image_crop_right_factor * display_horizontal_resolution;
        start_display_row_index = display_image_crop_top_factor * display_vertical_resolution;
        end_display_row_index = display_vertical_resolution - display_image_crop_bottom_factor * display_vertical_resolution;
        // offset of display
        display_column_offset = display_image_offset_x_factor * display_horizontal_resolution;
        display_row_offset = display_image_offset_y_factor * display_vertical_resolution;

        display_horizontal_resolution_bis =  max(horizontal_resolutions[1], horizontal_resolutions[3]);
        display_horizontal_resolution_bis = min(display_horizontal_resolution_bis, src_half_image_width);
        display_vertical_resolution_bis = max(vertical_resolutions[1], vertical_resolutions[3]);
        display_horizontal_resolution_offset_bis = display_horizontal_resolution_bis/NBSTRIPS;
        
        // first index of image
        start_image_column_index_bis = src_half_image_width + display_image_anchor_x_factor_bis * src_half_image_width - display_image_anchor_x_factor_bis * display_horizontal_resolution_bis;
        start_image_row_index_bis = display_image_anchor_y_factor_bis * src_image_height - display_image_anchor_y_factor_bis * display_vertical_resolution_bis;
        // crop of display
        start_display_column_index_bis = display_image_crop_left_factor_bis * display_horizontal_resolution_bis;
        end_display_column_index_bis = display_horizontal_resolution_bis - display_image_crop_right_factor_bis * display_horizontal_resolution_bis;
        start_display_row_index_bis = display_image_crop_top_factor_bis * display_vertical_resolution_bis;
        end_display_row_index_bis = display_vertical_resolution_bis - display_image_crop_bottom_factor_bis * display_vertical_resolution_bis;
        // offset of display
        display_column_offset_bis = display_image_offset_x_factor_bis * display_horizontal_resolution_bis;
        display_row_offset_bis = display_image_offset_y_factor_bis * display_vertical_resolution_bis;
}

void dualPersistenceOfVision(int target_display_rpm){
        if (display_horizontal_resolution > 0){
          // first strip position
          selectVspiMux(0); // 400-600 ns
          display_column_indexes[0] = getNanos(target_time_spacing_ns)/(float)target_time_spacing_ns  * display_horizontal_resolution;
          if (display_column_indexes[0] != prev_display_column_indexes[0]){
              offseted_display_column_index = (display_column_indexes[0] + display_column_offset) % display_horizontal_resolution;
              image_column_index  = start_image_column_index + offseted_display_column_index;
              if(
                dotstars_enabled[0]
                && start_display_column_index <= offseted_display_column_index && offseted_display_column_index <= end_display_column_index
                && image_column_index >= start_image_column_index && image_column_index < src_half_image_width
              ){
                setStripFromFrame(dotstar0, NUMPIXELS0 - display_vertical_resolution, NUMPIXELS0, start_display_row_index, end_display_row_index, image_column_index, start_image_row_index, display_row_offset);
              }
              else dotstar0 -> clear();
              dotstar0 -> show(); // before 9200000 - 930000 ns CPU@240Mhz  SPI@20Mhz -> after 280000 - 297000 ns CPU@240Mhz  SPI@20Mhz ( 3 x FASTER!)
              prev_display_column_indexes[0] = display_column_indexes[0];
          }
          // second strip position
          selectHspiMux(1); // 400-600 ns
          display_column_indexes[2] = getNanos(target_time_spacing_ns)/(float)target_time_spacing_ns  * display_horizontal_resolution;
          if (display_column_indexes[2] != prev_display_column_indexes[2]){
              offseted_display_column_index = ((display_column_indexes[2] + 2 * display_horizontal_resolution_offset + first_display_column_correction) % display_horizontal_resolution + display_column_offset) % display_horizontal_resolution;
              image_column_index =  start_image_column_index + offseted_display_column_index;
              if(
                dotstars_enabled[2] 
                && start_display_column_index <= offseted_display_column_index && offseted_display_column_index <= end_display_column_index 
                && image_column_index >= start_image_column_index && image_column_index < src_half_image_width
                ) {
                  setStripFromFrame(dotstar1, NUMPIXELS2 - display_vertical_resolution, NUMPIXELS2, start_display_row_index, end_display_row_index, image_column_index, start_image_row_index, display_row_offset);
                }
              else dotstar1 -> clear();
              dotstar1 -> show();  // before 9200000 - 930000 ns CPU@240Mhz  SPI@20Mhz -> after 280000 - 297000 ns CPU@240Mhz  SPI@20Mhz ( 3 x FASTER!)
              prev_display_column_indexes[2] = display_column_indexes[2];
          }
              
          if(pitchMode){
            dotstar0 -> clear();
            dotstar1 -> clear();
            dotstar0 -> show(); // before 9200000 - 930000 ns CPU@240Mhz  SPI@20Mhz -> after 280000 - 297000 ns CPU@240Mhz  SPI@20Mhz ( 3 x FASTER!)
            dotstar1 -> show(); // before 9200000 - 930000 ns CPU@240Mhz  SPI@20Mhz -> after 280000 - 297000 ns CPU@240Mhz  SPI@20Mhz ( 3 x FASTER!)
          }
        } 
          
        if (display_horizontal_resolution_bis > 0){
          // third strip
          selectVspiMux(1); // 400-600 ns
          display_column_indexes[1] = getNanos(target_time_spacing_ns)/(float)target_time_spacing_ns  * display_horizontal_resolution_bis;
          if (display_column_indexes[1] != prev_display_column_indexes[1]){
              offseted_display_column_index = ((display_column_indexes[1] + display_horizontal_resolution_offset_bis) % display_horizontal_resolution_bis + display_column_offset_bis) % display_horizontal_resolution_bis;
              image_column_index = start_image_column_index_bis + offseted_display_column_index;
              if(
                dotstars_enabled[1]
                && start_display_column_index_bis <= offseted_display_column_index && offseted_display_column_index <= end_display_column_index_bis
                && image_column_index >= start_image_column_index_bis && image_column_index < src_image_width
              ){
                
                setStripFromFrame(dotstar0, NUMPIXELS1 - display_vertical_resolution_bis, NUMPIXELS1, start_display_row_index_bis, end_display_row_index_bis, image_column_index, start_image_row_index_bis, display_row_offset_bis);
              }
              else dotstar0 -> clear();
              dotstar0 -> show(); // before 9200000 - 930000 ns CPU@240Mhz  SPI@20Mhz -> after 280000 - 297000 ns CPU@240Mhz  SPI@20Mhz ( 3 x FASTER!)
               prev_display_column_indexes[1] = display_column_indexes[1];
          }
          // fourth strip
          selectHspiMux(0); // 400-600 ns
          display_column_indexes[3] = getNanos(target_time_spacing_ns)/(float)target_time_spacing_ns  * display_horizontal_resolution_bis;
          if (display_column_indexes[3] != prev_display_column_indexes[3]){
              offseted_display_column_index = ((display_column_indexes[3] + 3 * display_horizontal_resolution_offset_bis + second_display_column_correction) % display_horizontal_resolution_bis  + display_column_offset_bis) % display_horizontal_resolution_bis;
              image_column_index  = start_image_column_index_bis + offseted_display_column_index;
              if(
                dotstars_enabled[3]
                && start_display_column_index_bis <= offseted_display_column_index && offseted_display_column_index <= end_display_column_index_bis
                && image_column_index >= start_image_column_index_bis && image_column_index < src_image_width
              ){
                setStripFromFrame(dotstar1, NUMPIXELS3 - display_vertical_resolution_bis, NUMPIXELS3, start_display_row_index_bis, end_display_row_index_bis, image_column_index, start_image_row_index_bis, display_row_offset_bis);
              }
              else dotstar1 -> clear();
              dotstar1 -> show(); // before 9200000 - 930000 ns CPU@240Mhz  SPI@20Mhz -> after 280000 - 297000 ns CPU@240Mhz  SPI@20Mhz ( 3 x FASTER!)
              prev_display_column_indexes[3] = display_column_indexes[3];
          }
        }
}

void handleLight(){
  if (!displayIsReset){
    // reset strip 0 and 1
      selectVspiMux(0);
      selectHspiMux(1);
      dotstar0 -> clear();
      dotstar1 -> clear();
      setStripFromColor(dotstar0, NUMPIXELS0 - reset_pixel, NUMPIXELS0, 0x110000);
      setStripFromColor(dotstar1, NUMPIXELS2 - reset_pixel, NUMPIXELS2, 0x110000);
      //dotstar0 -> setPixelColor(NUMPIXELS0 - reset_pixel, 0x880000); 
      //dotstar1 -> setPixelColor(NUMPIXELS3 - reset_pixel, 0x440044); 
      dotstar0 -> show(); 
      dotstar1 -> show(); 

      // reset strip 2 and 3
      selectVspiMux(1);
      selectHspiMux(0);
      dotstar0 -> clear();
      dotstar1 -> clear();
      setStripFromColor(dotstar0, NUMPIXELS1 - reset_pixel, NUMPIXELS1, 0x110000);
      setStripFromColor(dotstar1, NUMPIXELS3 - reset_pixel, NUMPIXELS3, 0x110000);
      //dotstar0 -> setPixelColor(NUMPIXELS1 - reset_pixel, 0x008800); 
      //dotstar1 -> setPixelColor(NUMPIXELS2 - reset_pixel, 0x000088); 
      dotstar0 -> show(); 
      dotstar1 -> show(); 

    if(displayToReset){

      delay(1000);
      Serial.print(NUMPIXELS0 - reset_pixel);
      Serial.print(" ");
      Serial.print(NUMPIXELS1 - reset_pixel);
      Serial.print(" ");
      Serial.print(NUMPIXELS2 - reset_pixel);
      Serial.print(" ");
      Serial.print(NUMPIXELS3 - reset_pixel);
      Serial.println();
      Serial2.print(RESET_CMD);
      Serial2.println(" ");
      displayToReset = false;
    }
  } 
  else {
      if (displayToTurnOff){
        //turn off strip 0 and 1
        dotstar0 -> clear();
        dotstar1 -> clear();
        selectVspiMux(0);
        selectHspiMux(0);
        delayNs(MUX_NANOSECONDS_DELAY);
        dotstar0 -> show(); 
        dotstar1 -> show(); 
        // turn off strip 2 and 3
        dotstar0 -> clear();
        dotstar1 -> clear();
        selectVspiMux(1);
        selectHspiMux(1);
        delayNs(MUX_NANOSECONDS_DELAY);
        dotstar0 -> show(); 
        dotstar1 -> show(); 

        displayToTurnOff = false;
      }
      if(displayActive) {
          // testLight();
          if(dualMode){
            dualPersistenceOfVision(display_rpm);
          } else {
            persistenceOfVision(display_rpm);
          }
      }
      
  }
}



void startCalibration(){
    calib_enabled = true;
    index_angular_speed = 0;
    angular_speed_is_triggered = false;
    angular_speed_is_reached = false;
    strips_done = false;
    display_done = false;
    index_strip = 0;
    
    for (int i = 0; i < NBSTRIPS; i++) dotstars_enabled [i] = false;

    src_image_width = DISPLAY_MAX_HORIZONTAL_RESOLUTION;
    src_image_height = DISPLAY_MAX_VERTICAL_RESOLUTION;
    byte red_8bits = 0b01100000;
    for (int i = 0; i < DISPLAY_MAX_NB_PIXELS; i++) {
            src_image_pixels[i] = red_8bits;
    }

}

void stopCalibration(){
    //Serial.println("stopCalibration");
    calib_enabled = true;
    index_angular_speed = NB_CALIB_ANGULAR_SPEED;
    angular_speed_is_triggered = false;
    angular_speed_is_reached = false;
    strips_done = true;
    display_done = true;
    index_strip = 0;
}


void showOneStripWithColor(int index, int color){
        dotstar0 -> clear();
        dotstar1 -> clear();
        selectVspiMux(0); // 400-600 ns
        selectHspiMux(1); // 400-600 ns
        if(index == 0){
          setStripFromColor(dotstar0, NUMPIXELS0 - vertical_resolutions[0], NUMPIXELS0, color);
        }
        if(index == 2){
          setStripFromColor(dotstar1, NUMPIXELS2 - vertical_resolutions[2], NUMPIXELS2, color);
        }
        dotstar0 -> show(); 
        dotstar1 -> show(); 
        // turn off strip 1 and 3
        dotstar0 -> clear();
        dotstar1 -> clear();
        selectVspiMux(1);
        selectHspiMux(0);
        if(index == 1){
          setStripFromColor(dotstar0, NUMPIXELS1 - vertical_resolutions[1], NUMPIXELS1, color);
        }
        if(index == 3){
          setStripFromColor(dotstar1, NUMPIXELS3 - vertical_resolutions[3], NUMPIXELS3, color);
        }
        dotstar0 -> show(); 
        dotstar1 -> show(); 
}
void calibration (){
  //Serial.println("calibration");
  if (display_done == true){
          if (index_angular_speed < NB_CALIB_ANGULAR_SPEED - 1){
            index_angular_speed = index_angular_speed + 1;
            angular_speed_is_triggered = false;
            angular_speed_is_reached = false;
            strips_done = false;
            display_done = false;
            index_strip = 0;
            Serial.printf("index_angular_speed: %d\n", index_angular_speed);
            Serial.printf("angular speed: %d\n",  calib_angular_speeds[index_angular_speed]);
          }
           else {
            target_spin_asked = 0;
            Serial.printf("Calibration command: spin %d\n", target_spin_asked);

            // LIVE
            spinner_command_enabled = false;
            spinner_keep_alive = true;
            calib_enabled = false;

            // // DEBUG
            // Serial.print(SPIN_CMD);
            // Serial.print(" ");
            // Serial.println(0);
            // calib_enabled = false;


            calib_to_publish = true;
            calibrated_display_rpm = 200;
            return;
           }
  }

  uint32_t angular_speed = calib_angular_speeds[index_angular_speed];
  if (angular_speed_is_triggered == false){ // trigger angular speed command
  
      target_spin_asked = angular_speed;
      
      // LIVE
      Serial.printf("Calibration command: spin %d\n", target_spin_asked);
      spinner_command_enabled = false;
      spinner_keep_alive = true;
      angular_speed_is_triggered = true;

      // //DEBUG
      // Serial.printf("Calibration command: spin %d\n", target_spin_asked);
      // spinner_command_enabled = false;
      // spinner_keep_alive = false;
      // angular_speed_is_triggered = true;
      // angular_speed_is_reached = true;

      calibrated_display_rpm  = (angular_speed > 1) ? RPM_TO_FPS_ERROR * angular_speed : 200;

  } else {
      // check if angular speed is reached
      if (angular_speed_is_reached == false){

        if (abs((int)angular_speed - (int)current_spin) <= 2) {
          angular_speed_is_reached = true;  
        } 
      } 
      // if angular speed is reached
      if (angular_speed_is_reached == true){ 
            // get time
            unsigned long current_time = millis();
            unsigned long strip_on_start_deltatime = current_time - strip_on_start_time;

            if (!strips_done){
                // if 6s have passed
                if (strip_on_start_deltatime > STRIP_MONOCOLOR_DURATION + STRIP_OFF_DURATION + STRIP_DUALCOLOR_DURATION + STRIP_OFF_DURATION + STRIP_DUALCOLOR_PITCH_DURATION + STRIP_OFF_DURATION){
                  // check that  next strip is in range
                  if (index_strip < NBSTRIPS){
                    for (int i = 0; i < NBSTRIPS; i++) dotstars_enabled [i] = false;
                    //dotstars_enabled [index_strip] = true;
                    //showOneStripWithColor(index_strip, 0x110000);
                    pitchMode = false;
                    strip_is_on = false;
                    strip_on_start_time = current_time;
                    //Serial.printf("strip %d turn on all pixels\n", index_strip);
                  }
                  // else strips are done 
                  else {
                    strip_on_start_time = current_time;
                    strips_done = true;
                    display_done = true;
                  }
                } else {
                  // turn on all pixels monocolor
                  if (strip_on_start_deltatime < STRIP_MONOCOLOR_DURATION){
                    if(!strip_is_on){
                      //Serial.printf("strip %d turn on mono color\n", index_strip);
                      dotstars_enabled [index_strip] = true;
                      pitchMode = false;
                      strip_is_on = true;
                    }
                  // turn off pixels monocolor
                  } else if (strip_on_start_deltatime < STRIP_MONOCOLOR_DURATION  + STRIP_OFF_DURATION){
                    if(strip_is_on){
                      //Serial.printf("strip %d turn off mono color\n", index_strip);
                      dotstars_enabled [index_strip] = false;
                      pitchMode = false;
                      strip_is_on = false;
                      // fill in red and blue
                      byte red_8bits =    0b01100000;
                      byte blue_8bits =   0b00000011;
                      byte color_8bits =  0b00000000;
                      for (int i = 0; i < src_image_width; i++) {
                        color_8bits = (i%2)? red_8bits : blue_8bits;
                        for (int j = 0; j < src_image_height; j++) {
                          int pixel_index = src_image_width * j + i; 
                          src_image_pixels[pixel_index] = color_8bits;
                        }
                      }
                      //Serial.println("filling image with blue and red bands");
                    }
                  }
                  // turn on all pixels dualcolor
                   else if (strip_on_start_deltatime < STRIP_MONOCOLOR_DURATION  + STRIP_OFF_DURATION + STRIP_DUALCOLOR_DURATION){
                    if(!strip_is_on){
                      //Serial.printf("strip %d turn on dual color\n", index_strip);
                      dotstars_enabled [index_strip] = true;
                      pitchMode = false;
                      strip_is_on = true;
                    }
                  }
                  // turn off all pixels dualcolor
                   else if (strip_on_start_deltatime < STRIP_MONOCOLOR_DURATION  + STRIP_OFF_DURATION + STRIP_DUALCOLOR_DURATION + STRIP_OFF_DURATION){
                      if(strip_is_on){
                        //Serial.printf("strip %d turn off dual color\n", index_strip);
                        dotstars_enabled [index_strip] = false;
                        pitchMode = false;
                        strip_is_on = false;
                        // save error
                        int calib_index = index_angular_speed * NBSTRIPS + index_strip;
                        //Serial.printf("calib_index: %u\n", calib_index);
                        calib_height[calib_index] = MIN_H + current_height;
                        calib_length[calib_index] =  initial_line + current_lines[index_strip];
                        calib_horizontal_resolution[calib_index] = horizontal_resolutions[index_strip];
                        calib_vertical_resolution[calib_index] = vertical_resolutions[index_strip];
                        calib_target_deltatime_spacing[calib_index] = target_delta_time_spacing_ns;

                        calib_deltatime_spacing_without_pitch[calib_index] = delta_time_spacing_ns;
                        calib_display_errors_without_pitch[calib_index] = missed_bands_per_frame;

                        //Serial.printf("CPU Frequency: %u, target delta time spacing: %u, delta time spacing: %u, missed_bands_per_frame: %d \n", CPU_Frequency_Mhz, target_delta_time_spacing_ns, delta_time_spacing_ns, missed_bands_per_frame);
                      }
                  }
                  // turn on all pixels dualcolor with pitch
                  else if (strip_on_start_deltatime < STRIP_MONOCOLOR_DURATION  + STRIP_OFF_DURATION + STRIP_DUALCOLOR_DURATION  + STRIP_OFF_DURATION + STRIP_DUALCOLOR_PITCH_DURATION){
                    if(!strip_is_on){
                      //Serial.printf("strip %d turn on dual color with pitch\n", index_strip);
                      dotstars_enabled [index_strip] = true;
                      pitchMode = true;
                      strip_is_on = true;
                    }
                  }
                  // turn on all pixels dualcolor with pitch
                  else if (strip_on_start_deltatime < STRIP_MONOCOLOR_DURATION  + STRIP_OFF_DURATION + STRIP_DUALCOLOR_DURATION  + STRIP_OFF_DURATION + STRIP_DUALCOLOR_PITCH_DURATION + STRIP_OFF_DURATION){
                    if(strip_is_on){
                        //Serial.printf("strip %d turn off dual color with pitch\n", index_strip);
                        dotstars_enabled [index_strip] = false;
                        pitchMode = false;
                        strip_is_on = false;
                        // save error
                        int calib_index = index_angular_speed * NBSTRIPS + index_strip;
                        // Serial.printf("calib_index: %d \n", calib_index);

                        calib_deltatime_spacing_with_pitch[calib_index] = delta_time_spacing_ns;
                        calib_display_errors_with_pitch[calib_index] = missed_bands_per_frame;

                        //Serial.printf("CPU Frequency: %u, target delta time spacing: %u, delta time spacing: %u, missed_bands_per_frame: %d \n", CPU_Frequency_Mhz, target_delta_time_spacing_ns, delta_time_spacing_ns, missed_bands_per_frame);
                        // fill with red
                        byte red_8bits = 0b01100000;
                        for (int i = 0; i < DISPLAY_MAX_NB_PIXELS; i++) {
                                src_image_pixels[i] = red_8bits;
                        }
                        //Serial.println("filling image with red bands");
                        index_strip = index_strip + 1;
                     }
                  }
                }
            }
           
      }
  }
}

void loop(void) {

  curr_time = millis();
  //handleServer();
  //handleState();  
  //handleSerial();
  // if (calib_enabled) {
  //   calibration();
  //   //persistenceOfVision(int target_display_rpm){
  //   persistenceOfVision(calibrated_display_rpm);
  // }
  if (display_rotation_delay > 0 && curr_time - prev_display_rotation_time > display_rotation_delay){
        display_image_offset_x_factor = display_image_offset_x_factor + 0.01;
        display_image_offset_x_factor_bis = display_image_offset_x_factor_bis + 0.01;
        // offset of display 1v
        display_column_offset = display_image_offset_x_factor * display_horizontal_resolution;
        display_row_offset = display_image_offset_y_factor * display_vertical_resolution;
        // offset of display 2 
        display_column_offset_bis = display_image_offset_x_factor_bis * display_horizontal_resolution_bis;
        display_row_offset_bis = display_image_offset_y_factor_bis * display_vertical_resolution_bis;
        prev_display_rotation_time = curr_time;
        //Serial.println(display_image_offset_x_factor);
  }
  handleLight();

  // if(millis() - prev_display_time > DISPLAY_DELAY){
  //   Serial.print("vertical_resolution (px):");
  //   Serial.print(vertical_resolution);
  //   Serial.print(", horizontal_resolution (px):");
  //   Serial.print(horizontal_resolution);
  //   Serial.print(", time_spacing (ns):");
  //   Serial.print(time_spacing);
  //   Serial.println();
  //   prev_display_time = millis();
  // }
  // delay(1);

}
