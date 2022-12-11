#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "mraa/spi.h"
#include "mraa/gpio.h"

#include <stdio.h>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"


#define SPI_BUS 0 

#define SPI_FREQ 12000000


mraa_spi_context spi;
mraa_gpio_context CS;
mraa_gpio_context r1;
mraa_gpio_context r2;
mraa_gpio_context RDY;
mraa_gpio_context BTN;

clock_t start, end;
double time_used;


// Performs a SPI transaction to the ESP32
// Sends a single frame stored in buffer
void sendFrame(uint8_t frame[])
{
	mraa_gpio_write(CS,0);

	mraa_spi_write_buf(spi, frame, 4096);

	mraa_gpio_write(CS,1);
}


// RGB value conversion shorthands
int to5(int value)
{
	return value*31/255;
}

int to6(int value)
{
	return value*63/255;
}



// preRender converts the RGB values and dumps them all in memory
void preRender(unsigned int frame_n, uint8_t output[], unsigned char *input)
{
	for(int i = 0; i < frame_n; i++)
	{
		for(int j = 0; j < 2048; j++)
		{
			unsigned char* pixelOffset = input + (int) (j%64 + 64 * floor(j/64) + i * 2048) * 3;

			output[ j 	  + 6144 * i ] = 	(uint8_t) to5( pixelOffset[0] );
			output[ j + 2048  + 6144 * i ] = 	(uint8_t) to6( pixelOffset[1] );
			output[ j + 4096  + 6144 * i ] = 	(uint8_t) to5( pixelOffset[2] );
		}
	}
}




int main(int argc, char** argv)
{
	// GPIO initialisations
	mraa_result_t status = MRAA_SUCCESS;

	mraa_deinit();
	mraa_init();
	
	mraa_spi_stop(spi);

	spi = mraa_spi_init(SPI_BUS);
	mraa_spi_frequency(spi, SPI_FREQ);
	mraa_spi_mode(spi, MRAA_SPI_MODE3);
	mraa_spi_bit_per_word(spi, 8);


	CS = mraa_gpio_init(19);
	mraa_gpio_dir(CS, MRAA_GPIO_OUT);

	mraa_gpio_write(CS, 1);
	sleep(0.25);
	mraa_gpio_write(CS, 0);
	sleep(0.25);
	mraa_gpio_write(CS, 1);


	
	r1 = mraa_gpio_init(7);
	r2 = mraa_gpio_init(11);
	RDY = mraa_gpio_init(22);
	BTN = mraa_gpio_init(21);
	
	mraa_gpio_dir(r1, MRAA_GPIO_IN);
	mraa_gpio_dir(r2, MRAA_GPIO_IN);
	mraa_gpio_dir(RDY, MRAA_GPIO_IN);
	mraa_gpio_dir(BTN, MRAA_GPIO_IN);


	// Opens GIF and dumps it in memory
	FILE* pf = fopen("mbta.gif", "rb");
	fseek(pf, 0L, SEEK_END);
	int size = (int) ftell(pf);
	void*buffer = malloc(size);
	fseek(pf, 0L, SEEK_SET);
	fread(buffer, size, 1, pf);
	fclose(pf);


	int *delays = nullptr;
	int x,y,z,comp2;
	

	// Loads the GIF and dumps all the RGB values in memory
	unsigned char *dataGIF = stbi_load_gif_from_memory( (stbi_uc*) buffer , size, &delays, &x,&y,&z, &comp2, 3);
	
	free(buffer);
	
	uint8_t rawFrames[z*2048*3];

	start = clock();
	preRender(z, rawFrames, dataGIF);
	end = clock();

	printf("%ld\n", end - start);
	
	printf("%d\n", z);
	fflush(stdout);
	uint8_t val[4096];
	
	clock_t rotaryTime = clock();
	bool lastA = mraa_gpio_read(r1);
	bool runLoop = true;
	bool debounceFlag = true;

	float rotCount = 0.7;

	int j = 0;
		
	clock_t frameTime = clock();
	clock_t btnTime = clock();
	clock_t transactionTime = 0;

	while(1){

		if( true )
		{
			rotaryTime = clock();
			bool a = mraa_gpio_read(r1);
			bool b = mraa_gpio_read(r2);
			
			// Rotary knob function, increments or decrements brightness based on direction of rotation
			if( a != lastA && a == 1 )
			{
				if( b!= a)
				{
					rotCount = rotCount < 0.91 ? rotCount + 0.01 : 1;
				}
				else
				{
					rotCount = rotCount - 0.01 > 0 ? rotCount - 0.01 : 0;
				}

				printf("%.2f\n", rotCount);
			}

			lastA = a;

		}
		
		// Button press pauses the GIF loop
		if(	mraa_gpio_read(BTN) == 0 && debounceFlag	)
		{	
			runLoop = !runLoop;
			btnTime = clock();
			debounceFlag = false;
		}

		if(debounceFlag == false && clock() - btnTime > 50000 && mraa_gpio_read(BTN) )
		{
			debounceFlag = true;
		}



		// Main rendering loop, loop proceeds after some frameTime to result in a fixed frame rate
		// TODO: render frame AFTER sending frame packet
		if(clock() - frameTime + transactionTime > 5000 && runLoop )
		{
			frameTime = clock();

		j ++;

		if(j >= z)
		{
			j = 0;
		}
		

			start = clock();
			for(int i = 0; i < 2048; i++)
			{
				// Modifying brightness value by multiplying all the RGB values
				// Can technically only dim the pixels
				unsigned char r = rawFrames[i  +  6144 * j]	  * 1	;
				unsigned char g = rawFrames[i  +  6144 * j + 2048]* 1	;
				unsigned char b = rawFrames[i  +  6144 * j + 4096]* 1	;

				r = r - r*rotCount	;
				g = g - g*rotCount	;
				b = b - b*rotCount	;

				val[2*i+1] = r << 3 | g >> 3;
				val[2*i] = (g & 0b000111) << 5 | b;
	
			}
			end = clock();

			// Performs a SPI transaction with the ESP32 if it is ready, otherwise skip the frame
			if( mraa_gpio_read(RDY) == 0  )
			{
				sendFrame(val);
			}
			transactionTime = clock() - frameTime;
		}


	}



	mraa_spi_stop;



	return 0;
}





