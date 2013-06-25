#include <stdio.h>
#include <fcntl.h>
 
int main(void)
{
  int fd;
  char gpio_buffer[10];
  char choice[10];

  fd = open( "/dev/dali_drv", O_RDWR );

  printf( "Value of fd is: %d", fd );

  if( fd < 0 )
  {
    printf("Cannot open device \t");
    printf(" fd = %d \n",fd);
    return 0;
  }

  printf("\nPlease enter choice: \t");
  scanf( "%s", choice );
  printf("Your choice is: %s \n", choice );
  write( fd, choice, 1 );
  read( fd, gpio_buffer, 1);
  printf("GPIO value is: %s \n", gpio_buffer );

  if( 0 != close(fd) )
  {
    printf("Could not close device\n");
  }

  return 0;
}
