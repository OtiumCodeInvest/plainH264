# plainH264
A basic h264 encoder / decoder in one file for test purposes, based on OpenH264

Encoder use a mixed profile aimed at low latency encodning and low memory usage

Decoder is only compliant with limited features used by encoder and is only intended as decoder for this encoder

Only current and previous frame are used for encoder

EncodeFrame always returns current encoded frame

Only I and P frames are supported and CABAC encoding. 4x4 blocks not supported

Width and height must be a multiple of 16

Test frames can be downloaded here:

https://drive.google.com/file/d/1Usu35bfkoNHoJWjVCQRZgr6R9d1G10QX/view?usp=drive_link

## Download and unzip test frames from command line
```
$ mkdir data/frames
$ curl -L "https://drive.usercontent.google.com/download?id=1Usu35bfkoNHoJWjVCQRZgr6R9d1G10QX&confirm=xxx" -o data/frames/bunny_720p.zip
$ unzip data/frames/bunny_720p.zip -d data/frames
```
## Build command on Linux
```
$ mkdir build
$ cd build
$ cmake ..
$ make
```

## Build command on Windows
```
$ mkdir build
$ cd build
$ cmake -DCMAKE_GENERATOR_PLATFORM=x64 ..
$ Open plainH264.sln project in Visual Studio and build
```

Alternative open as folder

<img width="1165" alt="screenshot-bunny" src="https://github.com/OtiumCodeInvest/H264/assets/98739117/be4be5b1-2ccf-413d-b19d-03427f30fbfb">

