#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <poll.h>

#include <zlib.h>
#include <glib.h>

#include <lcm/lcm.h>
#include <lcmtypes/kinect_depth_msg_t.h>
#include <lcmtypes/kinect_frame_msg_t.h>
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

#include <math.h>

#include "jpeg-utils-ijg.h"

#define DEPTH_VAL 8192

int window;

int width = 640;
int height = 480;

uint8_t* depth_img;
uint8_t* depth_uncompress_buffer;
uint8_t* rgb;

GLuint gl_depth_tex;
GLuint gl_rgb_tex;

lcm_t* lcm = NULL;

int doOutput = 0;
char outputPath[1024];

pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;

int pngFileIndex = 0;

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
  
  glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
  glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, depth_img);
	
  glBegin(GL_TRIANGLE_FAN);
  glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
  glTexCoord2f(0, 0); glVertex3f(0,0,0);
  glTexCoord2f(1, 0); glVertex3f(640,0,0);
  glTexCoord2f(1, 1); glVertex3f(640,480,0);
  glTexCoord2f(0, 1); glVertex3f(0,480,0);
  glEnd();
  
  glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
  //	if (current_format == FREENECT_VIDEO_RGB || current_format == FREENECT_VIDEO_YUV_RGB)
  glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb);
  //	else
  //		glTexImage2D(GL_TEXTURE_2D, 0, 1, 640, 480, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, rgb_front+640*4);
  
  glBegin(GL_TRIANGLE_FAN);
  glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
  glTexCoord2f(0, 0); glVertex3f(640,0,0);
  glTexCoord2f(1, 0); glVertex3f(1280,0,0);
  glTexCoord2f(1, 1); glVertex3f(1280,480,0);
  glTexCoord2f(0, 1); glVertex3f(640,480,0);
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
	glOrtho (0, 1280, 480, 0, -1.0f, 1.0f);
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
	glGenTextures(1, &gl_depth_tex);
	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glGenTextures(1, &gl_rgb_tex);
	glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	ReSizeGLScene(Width, Height);
}

uint16_t t_gamma[DEPTH_VAL];

static void
on_frame(const lcm_recv_buf_t* lcm, const char* channel, const kinect_frame_msg_t* msg, void* user)
{
  int i;
  png_bytep* row_pointers;
  // TODO check image width, height

  if(msg->image.image_data_nbytes) {
    if(msg->image.image_data_format == KINECT_IMAGE_MSG_T_VIDEO_RGB) {
      memcpy(rgb, msg->image.image_data, width * height * 3);
    } else if(msg->image.image_data_format == KINECT_IMAGE_MSG_T_VIDEO_RGB_JPEG) {
      jpegijg_decompress_8u_rgb (msg->image.image_data, msg->image.image_data_nbytes,
				 rgb, width, height, width * 3);
    }
  }

  if(msg->depth.depth_data_nbytes) {
    int i;
    const uint16_t* depth = NULL;
    if(msg->depth.compression == KINECT_DEPTH_MSG_T_COMPRESSION_NONE) {
      depth = (uint16_t*) msg->depth.depth_data;
    } else if (msg->depth.compression == KINECT_DEPTH_MSG_T_COMPRESSION_ZLIB) {
      unsigned long dlen = msg->depth.uncompressed_size;
      uncompress(depth_uncompress_buffer, &dlen, msg->depth.depth_data, msg->depth.depth_data_nbytes);
      depth = (uint16_t*) depth_uncompress_buffer;
      //printf("compressed size=%i\n", msg->depth.depth_data_nbytes);
      //printf("uncompressed size=%i\n", dlen);
    }
    
    pthread_mutex_lock( &mutex1 );
    int npixels = width * height;
    for (i=0; i<npixels; i++) {
      //printf("%i: %04X\n",  i, depth[i]);

#if 0
      int max = 2048;
      int min = 800;
      int p = (int)(255 * (depth[i] - min) / (max - min));
      depth_img[i*3 + 0] = p;
      depth_img[i*3 + 1] = p;
      depth_img[i*3 + 2] = p;
#else
      if ( depth[i] >= DEPTH_VAL ) {
        depth_img[3*i+0] = 0;
        depth_img[3*i+1] = 0;
        depth_img[3*i+2] = 0;
	continue;
      }
      int pval = t_gamma[depth[i]];
      int lb = pval & 0xff;
      switch (pval>>8) {
      case 0:
	depth_img[3*i+0] = 255;
	depth_img[3*i+1] = 255-lb;
	depth_img[3*i+2] = 255-lb;
	break;
      case 1:
	depth_img[3*i+0] = 255;
	depth_img[3*i+1] = lb;
	depth_img[3*i+2] = 0;
	break;
      case 2:
	depth_img[3*i+0] = 255-lb;
	depth_img[3*i+1] = 255;
	depth_img[3*i+2] = 0;
	break;
      case 3:
	depth_img[3*i+0] = 0;
	depth_img[3*i+1] = 255;
	depth_img[3*i+2] = lb;
	break;
      case 4:
	depth_img[3*i+0] = 0;
	depth_img[3*i+1] = 255-lb;
	depth_img[3*i+2] = 255;
	break;
      case 5:
	depth_img[3*i+0] = 0;
	depth_img[3*i+1] = 0;
	depth_img[3*i+2] = 255-lb;
	break;
      default:
	depth_img[3*i+0] = 0;
	depth_img[3*i+1] = 0;
	depth_img[3*i+2] = 0;
	break;
      }
#endif
    }

    row_pointers = (png_bytep*) malloc(sizeof(png_bytep) * height);
    for ( i = 0; i < height; i++ ) {
      row_pointers[i] = (png_bytep) malloc(3*width*2);
      memcpy(row_pointers[i] + 0*3*width,       rgb + i*3*width, width*3);
      memcpy(row_pointers[i] + 1*3*width, depth_img + i*3*width, width*3);
      //row_pointers[i] = depth_img + (i*3*width);
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
    //exit(-1);
  }
}

static void usage(const char* progname)
{
  fprintf (stderr, "Usage: %s [options]\n"
                   "\n"
                   "Options:\n"
                   "  -l URL      Specify LCM URL\n"
	           "  -c channel  Subscribe channel name\n"
                   "  -h          This help message\n", 
        	   "  -o path     Save frames to PNG\n",
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
  int y;

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
  int res;
  
  width = 640;
  height = 480;
  int npixels = width*height;
  
  depth_img = malloc(npixels*3);
  depth_uncompress_buffer = malloc(npixels*sizeof(uint16_t));
  rgb = malloc(npixels*3);
  
  int i;
  for (i=0; i<DEPTH_VAL; i++) {
    float v = i/(float)DEPTH_VAL;
    v = powf(v, 3)* 6;
    t_gamma[i] = v*6*256;
  }
  
  glutInit(&argc, argv);
  
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
  glutInitWindowSize(1280, 480);
  glutInitWindowPosition(0, 0);
  
  window = glutCreateWindow("Kinect LCM viewer");
  
  glutDisplayFunc(&DrawGLScene);
  glutIdleFunc(&DrawGLScene);
  glutReshapeFunc(&ReSizeGLScene);
  glutKeyboardFunc(&keyPressed);

  InitGL(1280, 480);

  char channelName[512];
  strcpy(channelName, "KINECT_FRAME");

  int c;
  char *lcm_url = NULL;
  while ((c = getopt (argc, argv, "hl:c:o:")) >= 0) {
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
    case 'h':
    case '?':
      usage(argv[0]);
    }
  }
  lcm = lcm_create(lcm_url);
  
  kinect_frame_msg_t_subscribe(lcm, channelName, on_frame, NULL);
  
  glutMainLoop();

  free(depth_img);
  free(depth_uncompress_buffer);
  free(rgb);
  lcm_destroy(lcm);
  
  return 0;
}
