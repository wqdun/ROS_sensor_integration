#include <stdio.h>
#include <setjmp.h>
#include <string.h>
#include <string>
#include <iostream>
#include <stdlib.h>
extern "C" {
    #include "jpeglib.h"
}

#define CalcPitch(b,w) ((((b)*(w)-1)/32+1)*4)

#define PUT_2B(array,offset,value)  \
        (array[offset] = (char) ((value) & 0xFF), \
         array[offset+1] = (char) (((value) >> 8) & 0xFF))
#define PUT_4B(array,offset,value)  \
        (array[offset] = (char) ((value) & 0xFF), \
         array[offset+1] = (char) (((value) >> 8) & 0xFF), \
         array[offset+2] = (char) (((value) >> 16) & 0xFF), \
         array[offset+3] = (char) (((value) >> 24) & 0xFF))

void write_bmp_header(j_decompress_ptr cinfo, FILE *output_file)
{
        char bmpfileheader[14];
        char bmpinfoheader[40];
        long headersize, bfSize;
        int bits_per_pixel, cmap_entries;


        int step;

        /* Compute colormap size and total file size */
        if (cinfo->out_color_space == JCS_RGB) {
                if (cinfo->quantize_colors) {
                        /* Colormapped RGB */
                        bits_per_pixel = 8;
                        cmap_entries = 256;
                } else {
                        /* Unquantized, full color RGB */
                        bits_per_pixel = 24;
                        cmap_entries = 0;
                }
        } else {
                /* Grayscale output.  We need to fake a 256-entry colormap. */
                bits_per_pixel = 8;
                cmap_entries = 256;
        }

        step = cinfo->output_width * cinfo->output_components;

        while ((step & 3) != 0) step++;

        /* File size */
        headersize = 14 + 40 + cmap_entries * 4; /* Header and colormap */

        bfSize = headersize + (long) step * (long) cinfo->output_height;

        /* Set unused fields of header to 0 */
        memset(bmpfileheader, 0, sizeof(bmpfileheader));
        memset(bmpinfoheader, 0 ,sizeof(bmpinfoheader));

        /* Fill the file header */
        bmpfileheader[0] = 0x42;/* first 2 bytes are ASCII 'B', 'M' */
        bmpfileheader[1] = 0x4D;
        PUT_4B(bmpfileheader, 2, bfSize); /* bfSize */
        /* we leave bfReserved1 & bfReserved2 = 0 */
        PUT_4B(bmpfileheader, 10, headersize); /* bfOffBits */

        /* Fill the info header (Microsoft calls this a BITMAPINFOHEADER) */
        PUT_2B(bmpinfoheader, 0, 40);   /* biSize */
        PUT_4B(bmpinfoheader, 4, cinfo->output_width); /* biWidth */
        PUT_4B(bmpinfoheader, 8, cinfo->output_height); /* biHeight */
        PUT_2B(bmpinfoheader, 12, 1);   /* biPlanes - must be 1 */
        PUT_2B(bmpinfoheader, 14, bits_per_pixel); /* biBitCount */
        /* we leave biCompression = 0, for none */
        /* we leave biSizeImage = 0; this is correct for uncompressed data */
        if (cinfo->density_unit == 2) { /* if have density in dots/cm, then */
                PUT_4B(bmpinfoheader, 24, (INT32) (cinfo->X_density*100)); /* XPels/M */
                PUT_4B(bmpinfoheader, 28, (INT32) (cinfo->Y_density*100)); /* XPels/M */
        }
        PUT_2B(bmpinfoheader, 32, cmap_entries); /* biClrUsed */
        /* we leave biClrImportant = 0 */

        if (fwrite(bmpfileheader, 1, 14, output_file) != (size_t) 14) {
                printf("write bmpfileheader error\n");
        }
        if (fwrite(bmpinfoheader, 1, 40, output_file) != (size_t) 40) {
                printf("write bmpinfoheader error\n");
        }

        if (cmap_entries > 0) {
        }
}

void write_pixel_data(j_decompress_ptr cinfo, unsigned char *output_buffer, FILE *output_file)
{
        int rows, cols;
        int row_width;
        int step;
        unsigned char *tmp = NULL;

        unsigned char *pdata;

        row_width = cinfo->output_width * cinfo->output_components;
        step = row_width;
        while ((step & 3) != 0) step++;

        pdata = (unsigned char *)malloc(step);
        memset(pdata, 0, step);

        tmp = output_buffer + row_width * (cinfo->output_height - 1);
        for (rows = 0; rows < cinfo->output_height; rows++) {
                for (cols = 0; cols < row_width; cols += 3) {
                        pdata[cols + 2] = tmp[cols + 0];
                        pdata[cols + 1] = tmp[cols + 1];
                        pdata[cols + 0] = tmp[cols + 2];
                }
                tmp -= row_width;
                fwrite(pdata, 1, step, output_file);
        }

        free(pdata);
}

int read_jpeg_file(const char *input_filename, const char *output_filename)
{
        struct jpeg_decompress_struct cinfo;
        struct jpeg_error_mgr jerr;
        FILE *input_file;
        FILE *output_file;
        JSAMPARRAY buffer;
        int row_width;

        unsigned char *output_buffer;
        unsigned char *tmp = NULL;

        cinfo.err = jpeg_std_error(&jerr);

        if ((input_file = fopen(input_filename, "rb")) == NULL) {
                fprintf(stderr, "can't open %s\n", input_filename);
                return -1;
        }

        if ((output_file = fopen(output_filename, "wb")) == NULL) {

                fprintf(stderr, "can't open %s\n", output_filename);
                return -1;
        }

        jpeg_create_decompress(&cinfo);

        /* Specify data source for decompression */
        jpeg_stdio_src(&cinfo, input_file);

        /* Read file header, set default decompression parameters */
        (void) jpeg_read_header(&cinfo, TRUE);

        /* Start decompressor */
        (void) jpeg_start_decompress(&cinfo);

        row_width = cinfo.output_width * cinfo.output_components;

        buffer = (*cinfo.mem->alloc_sarray)
                ((j_common_ptr) &cinfo, JPOOL_IMAGE, row_width, 1);

        write_bmp_header(&cinfo, output_file);

        output_buffer = (unsigned char *)malloc(row_width * cinfo.output_height);
        memset(output_buffer, 0, row_width * cinfo.output_height);
        tmp = output_buffer;

        /* Process data */
        while (cinfo.output_scanline < cinfo.output_height) {
                (void) jpeg_read_scanlines(&cinfo, buffer, 1);

                memcpy(tmp, *buffer, row_width);
                tmp += row_width;
        }

        write_pixel_data(&cinfo, output_buffer, output_file);

        free(output_buffer);

        (void) jpeg_finish_decompress(&cinfo);

        jpeg_destroy_decompress(&cinfo);

        /* Close files, if we opened them */
        fclose(input_file);
        fclose(output_file);


        return 0;
}

void GeneJpegFile(const char* jpegFileName, unsigned char* inputData,
    int nWidth, int nHeight, int nChannel, int nQuality)
{
    /* This struct contains the JPEG compression parameters and pointers to
    * working space (which is allocated as needed by the JPEG library).
    * It is possible to have several such structures, representing multiple
    * compression/decompression processes, in existence at once.  We refer
    * to any one struct (and its associated working data) as a "JPEG object".
    */
    struct jpeg_compress_struct cinfo;

    /* This struct represents a JPEG error handler.  It is declared separately
    * because applications often want to supply a specialized error handler
    * (see the second half of this file for an example).  But here we just
    * take the easy way out and use the standard error handler, which will
    * print a message on stderr and call exit() if compression fails.
    * Note that this struct must live as long as the main JPEG parameter
    * struct, to avoid dangling-pointer problems.
    */
    struct jpeg_error_mgr jerr;

    /* More stuff */
    FILE *outfile;                  /* target file */
    JSAMPROW row_pointer[1];        /* pointer to JSAMPLE row[s] */
    int     row_stride;             /* physical row width in image buffer */

    /* Step 1: allocate and initialize JPEG compression object */

    /* We have to set up the error handler first, in case the initialization
    * step fails.  (Unlikely, but it could happen if you are out of memory.)
    * This routine fills in the contents of struct jerr, and returns jerr's
    * address which we place into the link field in cinfo.
    */
    cinfo.err = jpeg_std_error(&jerr);

    /* Now we can initialize the JPEG compression object. */
    jpeg_create_compress(&cinfo);  /* Now we can initialize the JPEG compression object. */

    /* Step 2: specify data destination (eg, a file) */
    /* Note: steps 2 and 3 can be done in either order. */

    /* Here we use the library-supplied code to send compressed data to a
    * stdio stream.  You can also write your own code to do something else.
    * VERY IMPORTANT: use "b" option to fopen() if you are on a machine that
    * requires it in order to write binary files.
    */
    if ((outfile = fopen(jpegFileName, "wb")) == NULL)
    {
        fprintf(stderr, "can't open %s\n", jpegFileName);
        return;
    }
    jpeg_stdio_dest(&cinfo, outfile);

    /* Step 3: set parameters for compression */

    /* First we supply a description of the input image.
    * Four fields of the cinfo struct must be filled in:
    */
    cinfo.image_width = nWidth;                /* image width and height, in pixels */
    cinfo.image_height = nHeight;
    cinfo.input_components = nChannel;         /* # of color components per pixel */

    if (nChannel == 1)
    {
        cinfo.in_color_space = JCS_GRAYSCALE;  /* colorspace of input image */
    }
    else if (nChannel == 3)
    {
        cinfo.in_color_space = JCS_RGB;        /* colorspace of input image */
    }

    /* Now use the library's routine to set default compression parameters.
    * (You must set at least cinfo.in_color_space before calling this,
    * since the defaults depend on the source color space.)
    */
    jpeg_set_defaults(&cinfo);

    // Now you can set any non-default parameters you wish to.
    // Here we just illustrate the use of quality (quantization table) scaling:
    jpeg_set_quality(&cinfo, nQuality, TRUE); /* limit to baseline-JPEG values */

    /* Step 4: Start compressor */

    /* TRUE ensures that we will write a complete interchange-JPEG file.
    * Pass TRUE unless you are very sure of what you're doing.
    */
    jpeg_start_compress(&cinfo, TRUE);

    /* Step 5: while (scan lines remain to be written) */
    /*           jpeg_write_scanlines(...); */

    /* Here we use the library's state variable cinfo.next_scanline as the
    * loop counter, so that we don't have to keep track ourselves.
    * To keep things simple, we pass one scanline per call; you can pass
    * more if you wish, though.
    */
    row_stride = nWidth * nChannel; /* JSAMPLEs per row in image_buffer */

    while (cinfo.next_scanline < cinfo.image_height)
    {
        /* jpeg_write_scanlines expects an array of pointers to scanlines.
        * Here the array is only one element long, but you could pass
        * more than one scanline at a time if that's more convenient.
        */
        row_pointer[0] = &inputData[cinfo.next_scanline * row_stride];
        (void)jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    /* Step 6: Finish compression */
    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);

    /* After finish_compress, we can close the output file. */
    fclose(outfile);
}

void loadJPG(const std::string &inJpgName)
{
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr = { 0 };
    cinfo.err = jpeg_std_error(&jerr);

    FILE *inJpgFile = fopen(inJpgName.c_str(), "rb");
    if(!inJpgFile) {
        std::cout << "Failed to open file.\n";
        return;
    }

    jpeg_create_decompress(&cinfo);
    jpeg_stdio_src(&cinfo, inJpgFile);
    jpeg_read_header(&cinfo, TRUE);
    jpeg_start_decompress(&cinfo);

    const int row_width = cinfo.output_width * cinfo.output_components;

    JSAMPARRAY outBuffer = (*cinfo.mem->alloc_sarray)((j_common_ptr) &cinfo, JPOOL_IMAGE, row_width, 1);
    unsigned char *output_buffer = (unsigned char *)malloc(row_width * cinfo.output_height);
    memset(output_buffer, 0, row_width * cinfo.output_height);
    unsigned char *tmp = output_buffer;

    while(cinfo.output_scanline < cinfo.output_height) {
        (void)jpeg_read_scanlines(&cinfo, outBuffer, 1);

        memcpy(tmp, *outBuffer, row_width);
        tmp += row_width;
    }



    // JSAMPROW row_pointer[1];
    // size_t cbPitch;

    free(output_buffer);
    (void) jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    fclose(inJpgFile);




    // cbPitch=CalcPitch(24,g_uWidth);

    // //设置输出的颜色类型
    // cinfo.out_color_space=JCS_RGB;
    // jpeg_calc_output_dimensions(&cinfo);

    // //开始解压


    // row_pointer[0]=(uint8_t*)g_pBits;

    // //一行一行读取
    // while(cinfo.output_scanline<cinfo.output_height)
    // {
    //     jpeg_read_scanlines(&cinfo,row_pointer,1);
    //     row_pointer[0]+=cbPitch;
    // }

    // //读取结束
    // jpeg_finish_decompress(&cinfo);

    // //释放资源
    // jpeg_destroy_decompress(&cinfo);

    // //关闭文件
    // fclose(fp);
}




int main(int argc, char *argv[])
{
    loadJPG("./11111.jpg");
        if (argc < 3) {
                read_jpeg_file("472.jpeg", "tt.bmp");
        } else {
                read_jpeg_file(argv[1], argv[2]);
        }
        return 0;
}