#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <poll.h>

#include <zlib.h>
#include <glib.h>

#include <lcm/lcm.h>
#include <lcmtypes/multisense.h>
#include <lcmtypes/bot_core_image_t.h>
#include <pthread.h>
#include <png.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <stdbool.h>
#include <math.h>

#include "jpeg-utils-ijg.h"

#define DEPTH_VAL 8192

int window;

// 800x800 - vrc
// 1024x544 - drc (real sensor)
// 1024x1024 - extended HFOV real sensor
int width = 1024;
int height = 1024;

bool rotateImg = false;

GLuint gl_leftpane_tex;
int32_t left_color_format;
uint8_t* left_img_data;


GLuint gl_rightpane_tex;
int rightpane_is_depth; // false = depth | true = color

int32_t right_color_format;
uint8_t* right_img_data;

uint8_t* depth_img_data;
uint8_t* depth_uncompress_buffer;


lcm_t* lcm = NULL;

int doOutput = 0;
char outputPath[1024];

pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;

int pngFileIndex = 0;

// Explicit declaration
void write_png_file(char* file_name, int width, int height, png_bytep* row_pointers);

void DrawGLScene()
{
  struct pollfd pfd = { lcm_get_fileno(lcm), POLLIN, 0 };
  int status = poll (&pfd, 1, 10);
  if (status > 0) {
    lcm_handle(lcm);
  }

  pthread_mutex_lock( &mutex1 );

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  glEnable(GL_TEXTURE_2D);

  glBindTexture(GL_TEXTURE_2D, gl_leftpane_tex);
  if ((left_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB) ||
      (left_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG )){
    glTexImage2D(GL_TEXTURE_2D, 0, 3, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, left_img_data);
  }else if (left_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY){
    glTexImage2D(GL_TEXTURE_2D, 0, 1, width, height, 0, GL_RED, GL_UNSIGNED_BYTE, left_img_data);
  }

  glBegin(GL_TRIANGLE_FAN);
  glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
  glTexCoord2f(0, 0); glVertex3f(0,0,0);
  glTexCoord2f(1, 0); glVertex3f(width,0,0);
  glTexCoord2f(1, 1); glVertex3f(width,height,0);
  glTexCoord2f(0, 1); glVertex3f(0,height,0);
  glEnd();

  glBindTexture(GL_TEXTURE_2D, gl_rightpane_tex);
  if (rightpane_is_depth){
  glTexImage2D(GL_TEXTURE_2D, 0, 3, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, depth_img_data);
  } else{

    if ((right_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB) ||
        (right_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG )){
      glTexImage2D(GL_TEXTURE_2D, 0, 3, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, right_img_data);
    }else if (right_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY){
      glTexImage2D(GL_TEXTURE_2D, 0, 1, width, height, 0, GL_RED, GL_UNSIGNED_BYTE, right_img_data);
    }

  }

  glBegin(GL_TRIANGLE_FAN);
  glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
  glTexCoord2f(0, 0); glVertex3f(width,0,0);
  glTexCoord2f(1, 0); glVertex3f(width*2,0,0);
  glTexCoord2f(1, 1); glVertex3f(width*2,height,0);
  glTexCoord2f(0, 1); glVertex3f(width,height,0);
  glEnd();

  glutSwapBuffers();

  pthread_mutex_unlock( &mutex1 );
}

void keyPressed(unsigned char key, int x, int y)
{
	if (key == 27) {
		glutDestroyWindow(window);
	}
}

void ReSizeGLScene(int Width, int Height)
{
	glViewport(0,0,Width,Height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho (0, width*2, height, 0, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
}

void InitGL(int Width, int Height)
{
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0);
	glDepthFunc(GL_LESS);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glShadeModel(GL_SMOOTH);

  if (rotateImg) {
    glMatrixMode(GL_TEXTURE);
    glTranslatef(1.0f, 0.0f, 0.0f);
    glRotatef(180.0f, 0.0f, 0.0f, 1.0f);
    glMatrixMode(GL_MODELVIEW);
  }

	glGenTextures(1, &gl_rightpane_tex);
	glBindTexture(GL_TEXTURE_2D, gl_rightpane_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glGenTextures(1, &gl_leftpane_tex);
	glBindTexture(GL_TEXTURE_2D, gl_leftpane_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	ReSizeGLScene(Width, Height);
}

uint16_t t_gamma[DEPTH_VAL];

static void
on_frame(const lcm_recv_buf_t* lcm, const char* channel, const multisense_images_t* msg, void* user)
{
  png_bytep* row_pointers;
  // TODO check image width, height
  width = msg->images[0].width;
  height = msg->images[0].height;

  if(msg->images[0].size) {
    left_color_format =  msg->images[0].pixelformat;
    if(left_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB) {
      memcpy(left_img_data, msg->images[0].data, width * height * 3);
    } else if(left_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG) {
      jpegijg_decompress_8u_rgb (msg->images[0].data, msg->images[0].size,
            left_img_data, width, height, width * 3);
    }else if (left_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY) {
      memcpy(left_img_data, msg->images[0].data, width * height);
    }else{
      printf("First Image Format Not Understood\n");
    }
  }

  if( msg->image_types[1] == MULTISENSE_IMAGES_T_RIGHT ){
    rightpane_is_depth = 0;

    right_color_format =  msg->images[1].pixelformat;
    if(right_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB) {
      memcpy(right_img_data, msg->images[1].data, width * height * 3);
    } else if(right_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG) {
      jpegijg_decompress_8u_rgb (msg->images[1].data, msg->images[1].size,
            right_img_data, width, height, width * 3);
    }else if (right_color_format == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY) {
      memcpy(right_img_data, msg->images[1].data, width * height);
    }else{
      printf("Second Image Format Not Understood\n");
    }

  }else if ( (msg->image_types[1] == MULTISENSE_IMAGES_T_DISPARITY || msg->image_types[1] == MULTISENSE_IMAGES_T_DISPARITY_ZIPPED) ||
             (msg->image_types[1] == MULTISENSE_IMAGES_T_DEPTH_MM  || msg->image_types[1] == MULTISENSE_IMAGES_T_DEPTH_MM_ZIPPED) ){
    rightpane_is_depth = 1;

    int i;
    const uint16_t* depth = NULL;
    int scaling=10;
    if(msg->image_types[1] == MULTISENSE_IMAGES_T_DISPARITY) {
      depth = (uint16_t*) msg->images[1].data;
    }else if (msg->image_types[1] == MULTISENSE_IMAGES_T_DISPARITY_ZIPPED ) {
      unsigned long dlen = width*height*2 ;//msg->depth.uncompressed_size;
      uncompress(depth_uncompress_buffer, &dlen, msg->images[1].data, msg->images[1].size);
      depth = (uint16_t*) depth_uncompress_buffer;
    }else if (msg->image_types[1] == MULTISENSE_IMAGES_T_DEPTH_MM ) {
      depth = (uint16_t*) msg->images[1].data;
      scaling=2;
    }else if (msg->image_types[1] == MULTISENSE_IMAGES_T_DEPTH_MM_ZIPPED ) {
      unsigned long dlen = width*height*2 ;//msg->depth.uncompressed_size;
      uncompress(depth_uncompress_buffer, &dlen, msg->images[1].data, msg->images[1].size);
      depth = (uint16_t*) depth_uncompress_buffer;
      scaling=2;
    }else{
      printf("Second Image Format Not Understood [B]\n");
    }

    pthread_mutex_lock( &mutex1 );
    int npixels = width * height;
    for (i=0; i<npixels; i++) {
      //printf("%i: %04X\n",  i, depth[i]);

      if ( scaling*depth[i] >= DEPTH_VAL ) {
        depth_img_data[3*i+0] = 0;
        depth_img_data[3*i+1] = 0;
        depth_img_data[3*i+2] = 0;
        continue;
      }

      int pval = t_gamma[scaling*depth[i]];
      int lb = pval & 0xff;
      switch (pval>>8) {
      case 0:
        depth_img_data[3*i+0] = 255;
        depth_img_data[3*i+1] = 255-lb;
        depth_img_data[3*i+2] = 255-lb;
        break;
            case 1:
        depth_img_data[3*i+0] = 255;
        depth_img_data[3*i+1] = lb;
        depth_img_data[3*i+2] = 0;
        break;
            case 2:
        depth_img_data[3*i+0] = 255-lb;
        depth_img_data[3*i+1] = 255;
        depth_img_data[3*i+2] = 0;
        break;
            case 3:
        depth_img_data[3*i+0] = 0;
        depth_img_data[3*i+1] = 255;
        depth_img_data[3*i+2] = lb;
        break;
            case 4:
        depth_img_data[3*i+0] = 0;
        depth_img_data[3*i+1] = 255-lb;
        depth_img_data[3*i+2] = 255;
        break;
            case 5:
        depth_img_data[3*i+0] = 0;
        depth_img_data[3*i+1] = 0;
        depth_img_data[3*i+2] = 255-lb;
        break;
            default:
        depth_img_data[3*i+0] = 0;
        depth_img_data[3*i+1] = 0;
        depth_img_data[3*i+2] = 0;
        break;
      }
    }

    row_pointers = (png_bytep*) malloc(sizeof(png_bytep) * height);
    for ( i = 0; i < height; i++ ) {
      row_pointers[i] = (png_bytep) malloc(3*width*2);
      memcpy(row_pointers[i] + 0*3*width, depth_img_data + i*3*width, width*3);
      memcpy(row_pointers[i] + 1*3*width, depth_img_data + i*3*width, width*3);
    }
    if ( doOutput ) {
      char filename[1024];
      sprintf(filename, "%s/img%08i.png", outputPath, pngFileIndex++);
      write_png_file(filename, width*2, height, row_pointers);
    }

    for ( i = 0; i < height; i++ ) {
      free(row_pointers[i]);
    }
    free(row_pointers);

    pthread_mutex_unlock( &mutex1 );
  }else{
      printf("Second Image Format Not Understood [C]\n");
  }
}

static void usage(const char* progname)
{
  fprintf (stderr, "Usage: %s [options]\n"
                   "\n"
                   "Options:\n"
                   "  -l URL      Specify LCM URL\n"
	              "  -c channel  Subscribe channel name\n"
        	         "  -o path     Save frames to PNG\n",
                   "  -p          Use previous resolution (1024x544)\n",
                   "  -r          Rotate/flip image(s) by 180 degrees\n",
                   "  -h          This help message\n",
                   g_path_get_basename(progname));
  exit(1);
}

void abort_(const char * s, ...)
{
        va_list args;
        va_start(args, s);
        vfprintf(stderr, s, args);
        fprintf(stderr, "\n");
        va_end(args);
        abort();
}

void write_png_file(char* file_name, int width, int height, png_bytep* row_pointers )
{
  png_byte color_type = PNG_COLOR_TYPE_RGB;
  png_byte bit_depth = 8;

  /* create file */
  FILE *fp = fopen(file_name, "wb");
  if (!fp)
    abort_("[write_png_file] File %s could not be opened for writing", file_name);


  /* initialize stuff */
  png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

  if (!png_ptr)
    abort_("[write_png_file] png_create_write_struct failed");

  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr)
    abort_("[write_png_file] png_create_info_struct failed");

  if (setjmp(png_jmpbuf(png_ptr)))
    abort_("[write_png_file] Error during init_io");

  png_init_io(png_ptr, fp);


  /* write header */
  if (setjmp(png_jmpbuf(png_ptr)))
    abort_("[write_png_file] Error during writing header");

  png_set_IHDR(png_ptr, info_ptr, width, height,
	       bit_depth, color_type, PNG_INTERLACE_NONE,
	       PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

  png_write_info(png_ptr, info_ptr);


  /* write bytes */
  if (setjmp(png_jmpbuf(png_ptr)))
    abort_("[write_png_file] Error during writing bytes");

  png_write_image(png_ptr, row_pointers);


  /* end write */
  if (setjmp(png_jmpbuf(png_ptr)))
    abort_("[write_png_file] Error during end of write");

  png_write_end(png_ptr, NULL);

  /*
  for (y=0; y<height; y++)
    free(row_pointers[y]);
  free(row_pointers);
*/

  fclose(fp);
}


int main(int argc, char **argv)
{

  char channelName[512];
  strcpy(channelName, "CAMERA");
  char *lcm_url = NULL;

  int c;
  while ((c = getopt (argc, argv, "hl:c:o:p:r")) >= 0) {
    switch (c) {
    case 'l':
      lcm_url = strdup(optarg);
      printf("Using LCM URL \"%s\"\n", lcm_url);
      break;
    case 'c' :
      strcpy(channelName, optarg);
      printf("Listening to channel %s\n", channelName);
      break;
    case 'o' :
      strcpy(outputPath, optarg);
      printf("Capturing PNGs to %s\n", outputPath);
      doOutput = 1;
      break;
    case 'p' :
      width = 1024;
      height = 544;
      break;
    case 'r' :
      rotateImg = true;
      break;
    case 'h':
    case '?':
      usage(argv[0]);
    }
  }
  lcm = lcm_create(lcm_url);

  int npixels = width*height;

  depth_img_data = malloc(npixels*3);
  depth_uncompress_buffer = malloc(npixels*sizeof(uint16_t));
  left_img_data = malloc(npixels*3);
  right_img_data = malloc(npixels*3);
  rightpane_is_depth = 0;

  int i;
  for (i=0; i<DEPTH_VAL; i++) {
    float v = i/(float)DEPTH_VAL;
    v = powf(v, 3)* 6;
    t_gamma[i] = v*6*256;
  }

  glutInit(&argc, argv);

  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
  //glutInitWindowSize(width*2, height);
  glutInitWindowSize(width, height/2);
  glutInitWindowPosition(0, 0);

  window = glutCreateWindow("Multisense LCM viewer");

  glutDisplayFunc(&DrawGLScene);
  glutIdleFunc(&DrawGLScene);
  glutReshapeFunc(&ReSizeGLScene);
  glutKeyboardFunc(&keyPressed);

  InitGL(width*2, height);


  multisense_images_t_subscribe(lcm, channelName, on_frame, NULL);


  glutMainLoop();
  free(depth_img_data);
  free(depth_uncompress_buffer);
  free(left_img_data);
  lcm_destroy(lcm);

  return 0;
}
