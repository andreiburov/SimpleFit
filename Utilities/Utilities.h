#pragma once

//image_name
int grabcut(int argc, char** argv);
//-w 9 -h 6 -s 26
int calibration(int argc, char** argv);
// 'c' - capture, 'p' - print
int TakeSnapshot(int argc, char** argv);

int RecordVideo(int argc, char** argv);
// smpl.posedirs.txt smpl.posedirs
int Convert(int argc, char** argv);
// smpl.posedirs smpl.posedirs.txt
void ConvertBack(int argc, char** argv);

void FreeImageExample();