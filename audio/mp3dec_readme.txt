Instructions for using the mp3dec library:

1. Download the source code of the Android P version, where the path of the mp3dec source is /frameworks/av/media/libstagefright/codecs/mp3dec;

2. Copy the src and include folders in the mp3dec directory of Android source code to the /mpp/sample/audio/mp3dec/OpenSource/mp3dec directory.

3. In the Linux server, enter the /mpp/sample/audio/mp3dec/OpenSource directory, and patch it, the command is as follows:
    cd /mpp/sample/audio/mp3dec/OpenSource
    patch -p1 < ./mp3dec.patch

4. In the /mpp/sample/audio/mp3dec directory, type the make command to generate libmp3dec.so and libmp3dec.a in the /mpp/sample/audio/mp3dec/lib directory.
    cd /mpp/sample/audio/mp3dec
    make

5. Link the required dynamic/static mp3dec library in the Makefile that generates the executable file. The corresponding header file is in the /mpp/sample/audio/mp3dec/include directory.

6. If you use the dynamic mp3dec library, you need to copy libmp3dec.so to the /usr/lib directory of the board.

7. If you want to use the mp3dec function in the sample, you should assign HI_SUPPORT_MP3DEC in the /mpp/sample/audio/Makefile to YES after compiling the mp3dec library, and then configure the corresponding parameters.

note:

1. If you need to test the mp3 decoding function, you must provide mp3 file by yourself.

2. The sample does not support mp3 encoding.