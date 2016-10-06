#define _USE_MATH_DEFINES

#include "urg_sensor.h"
#include "urg_utils.h"
#include <stdlib.h>
#include <math.h>

/*

エラー	1	error LNK2019: 未解決の外部シンボル "public: __thiscall urg_unko::urg_unko(void)" (??0urg_unko@@QAE@XZ) が関数
"public: __thiscall urg_mapping::urg_mapping(void)" (??0urg_mapping@@QAE@XZ) で参照されました。	c:\Users\user\documents\visual studio 2013\Projects\mappinng\mappinng\Source2.obj	mappinng


int main(void)
{
urg_t urg;
int ret;
long *length_data;
int length_data_size;
const int scan_times = 123;
int i;

// \~japanese "COM1" は、センサが認識されているデバイス名にする必要がある
const char connect_device[] = "COM1";
const long connect_baudrate = 115200;

// \~japanese センサに対して接続を行う。
ret = urg_open(&urg, URG_SERIAL, connect_device, connect_baudrate);
// \todo check error code

// \~japanese データ受信のための領域を確保する
length_data = (long *)malloc(sizeof(long) * urg_max_data_size(&urg));
// \todo check length_data is not NULL

// \~japanese 距離データの計測開始。
ret = urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
// \todo check error code

// \~japanese センサから距離データを取得する。
for (i = 0; i < scan_times; ++i) {
length_data_size = urg_get_distance(&urg, length_data, NULL);
}	// \todo process length_data array

// \~japanese センサとの接続を閉じる。
urg_close(&urg);

return 0;
}*/
#define def_obstacle_detection(p_data,  urg) obstacle_detection( p_data,  urg, sizeof((p_data[0]))/sizeof(p_data[0][0]), sizeof((p_data))/sizeof(p_data[0])) 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
void obstacle_detection(int **p_data, urg_t urg)
{
	//printf("(%d,%d)\n", x_division_coun, y_division_coun);
	int URG_L = 4000;															//URGの測定距離
	int length_data_size;														//測定点の数
	long *length_data;															//測定距離の格納先
	long min_distance, max_distance;												//測定可能距離
	int i, j;
	int p_x, p_y;
	int old_p_x=0, old_p_y=0;
	double slope;
	long Disx_resolution = URG_L / 20;			//x軸方向の分割数
	long Disy_resolution = URG_L /20;	//y軸方向の分割数
	int center[2] = { 0, 20 };
	double border_x=0, border_y = 0, old_border_x, old_border_y;
																	//配列の座標
	
	length_data = (long *)malloc(sizeof(long) * urg_max_data_size(&urg));
	urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
	length_data_size = urg_get_distance(&urg, length_data, NULL);				//測定した点群の距離取得
	urg_distance_min_max(&urg, &min_distance, &max_distance);					//測定可能範囲の定義

	for (i = 0; i < length_data_size; ++i) {

		// \~japanese その距離データのラジアン角度を求め、X, Y の座標値を計算する
		double radian;
		long length;
		int x;
		int y;

		radian = urg_index2rad(&urg, i);
		//printf("%lf\n", radian);
		length = length_data[i];
		// \todo check length is valid

		if ((length > min_distance) && (length < max_distance))
		{
			if (radian<M_PI / 2 && radian>-M_PI / 2)
			{
				p_x = (length * cos(radian)) / Disx_resolution;
				p_y = ((length * sin(radian)) + URG_L) / Disy_resolution;
				
				
				p_data[p_y][p_x] = 1;
				old_border_x = p_x;
				old_border_y = p_y;
				slope = (double)(p_x + 0.5 - center[0]) / (p_y + 0.5 - center[1]);
				if (abs(slope)<=1)
				{
					if (slope < 0)
					{//yマイナス方向に進行
						
						while (old_border_x >= 0)
						{

							//Sleep(100);
							border_y = old_border_x / slope + center[1];
							for (old_border_y = floor(old_border_y); old_border_y < ceil(border_y); old_border_y++)
							{
								
								
								if (old_border_y >= 0 && p_data[(int)old_border_y][(int)old_border_x] == 2)
									p_data[(int)old_border_y][(int)old_border_x] = 0;
							}
							old_border_y = border_y;
							old_border_x--;
						
						
						}
						
						/*
						border_y = p_y-1;
						border_x = (border_y - center[1]) * slope;
						if (p_data[p_y - 1][p_x]!=1)
							p_data[(int)border_y][p_x] = 2;
						
						
							border = border_x;
							border=ceil(border);
							printf("%lf", (border - border_x) / 1);
							//while (border_y==0||border_x==19)
							{
							
							if (abs(slope)<(border - border_x))
							{
								printf("(%d,%d)", p_x, p_y);
								printf("(%lf,%lf,%lf,%lf)", slope, border_x, border_y, border);
								border_y--;
								border_x = (border_y - center[1]) * slope;
								p_data[(int)border_y][(int)border] = 2;
								border = border_x;
								border = ceil(border);
							}else
							{
								
							}
						}
						*/
					}
					else
					{//yプラス方向に進行
						
						while (old_border_x >= 0)
						{
							
							border_y = old_border_x / slope + center[1];

							for (old_border_y = floor(old_border_y); old_border_y >= floor(border_y); old_border_y--)
							{
								
								if (old_border_y <= 39 && p_data[(int)old_border_y][(int)old_border_x] == 2)
									p_data[(int)old_border_y][(int)old_border_x] = 0;
							}
							old_border_y = border_y;
							old_border_x--;


						}
					}
					
				}
				else
				{//xプラス方向に進行
					
					if (slope == 1)
					{
						
						for (; old_border_x > 0; old_border_x--, old_border_y--)
						{
							
							if ( p_data[(int)old_border_y][(int)old_border_x] == 2)
							{
								
								p_data[(int)old_border_y][(int)old_border_x] = 0;
							}
						}
					}
					if (slope < 0)
					{
						
						while (old_border_x >= 0)
						{
							//Sleep(100);
							border_x = (old_border_y + 1- center[1]) * slope + center[0];

							for (old_border_x = floor(old_border_x); old_border_x >= floor(border_x); old_border_x--)
							{
								
								if (old_border_x <= 19 && p_data[(int)old_border_y][(int)old_border_x] == 2)
								{
									
									p_data[(int)old_border_y][(int)old_border_x] = 0;
								}

							}
							old_border_y++;
							old_border_x = border_x;

							
						}
					}
					else
					{
						printf("\n(%d,%d)", (int)old_border_x, (int)old_border_y);
						while (old_border_x >= 0)
						{
							//Sleep(100);
							border_x = (old_border_y + 1 - center[1]) * slope + center[0];

							for (old_border_x = floor(old_border_x); old_border_x >= floor(border_x); old_border_x--)
							{

								if (old_border_x <= 19 && p_data[(int)old_border_y][(int)old_border_x] == 2)
								{

									p_data[(int)old_border_y][(int)old_border_x] = 0;
								}

							}
							old_border_y--;
							old_border_x = border_x;


						}
					}
				}
				
				if (old_p_x != p_x || old_p_y != p_y)
				{
					//printf("(%d,%d)", p_x, p_y);
					//printf("(%lf,%lf,%lf)\n", slope, border_x, border_y);
					
				}
				old_p_x = p_x;
				old_p_y = p_y;

			}

		}

	}
	printf("\n");
	for (j = 0; j <20 ; j++)
	{
		for (i = 0; i <40 ; i++)
		{
			printf("%d,", p_data[i][j]);
		}
	}
	
}
void obstacle_detection2(int **p_data, urg_t urg, int x_division_coun, int y_division_coun)
{
	printf("(%d,%d)\n", x_division_coun, y_division_coun);
	int URG_L = 4000;															//URGの測定距離
	int length_data_size;														//測定点の数
	long *length_data;															//測定距離の格納先
	long min_distance, max_distance;												//測定可能距離
	int i, j;
	int p_x, p_y;
	int old_p_x = 0, old_p_y = 0;
	double slope;
	long Disx_resolution = URG_L / 20;			//x軸方向の分割数
	long Disy_resolution = URG_L / 20;	//y軸方向の分割数
	int center[2] = { 0, 20 };
	double border_x = 0, border_y = 0, old_border_x, old_border_y;
	//配列の座標

	length_data = (long *)malloc(sizeof(long) * urg_max_data_size(&urg));
	urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
	length_data_size = urg_get_distance(&urg, length_data, NULL);				//測定した点群の距離取得
	urg_distance_min_max(&urg, &min_distance, &max_distance);					//測定可能範囲の定義

	for (i = 0; i < length_data_size; ++i) {

		// \~japanese その距離データのラジアン角度を求め、X, Y の座標値を計算する
		double radian;
		long length;
		int x;
		int y;

		radian = urg_index2rad(&urg, i);
		//printf("%lf\n", radian);
		length = length_data[i];
		// \todo check length is valid

		if ((length > min_distance) && (length < max_distance))
		{
			if (radian<M_PI / 2 && radian>-M_PI / 2)
			{
				p_x = (length * cos(radian)) / Disx_resolution;
				p_y = ((length * sin(radian)) + URG_L) / Disy_resolution;


				p_data[p_y][p_x] = 1;
				old_border_x = p_x;
				old_border_y = p_y;
				slope = (double)(p_x + 0.5 - center[0]) / (p_y + 0.5 - center[1]);
				if (abs(slope)<1)
				{
					if (slope < 0)
					{//yマイナス方向に進行

						while (old_border_y > 0)
						{


							border_y = old_border_x/ slope + center[1];

							for (old_border_y = floor(old_border_y); old_border_y >= ceil(border_y); old_border_y++)
							{
								printf("%lf,%lf,%lf", old_border_x, old_border_y, border_y);
								if (old_border_y >= 0 && p_data[(int)old_border_y][(int)old_border_x] != 1)
									p_data[(int)old_border_y][(int)old_border_x] = 0;
							}
							old_border_y = border_y;
							old_border_x--;

						}

						/*
						border_y = p_y-1;
						border_x = (border_y - center[1]) * slope;
						if (p_data[p_y - 1][p_x]!=1)
						p_data[(int)border_y][p_x] = 2;


						border = border_x;
						border=ceil(border);
						printf("%lf", (border - border_x) / 1);
						//while (border_y==0||border_x==19)
						{

						if (abs(slope)<(border - border_x))
						{
						printf("(%d,%d)", p_x, p_y);
						printf("(%lf,%lf,%lf,%lf)", slope, border_x, border_y, border);
						border_y--;
						border_x = (border_y - center[1]) * slope;
						p_data[(int)border_y][(int)border] = 2;
						border = border_x;
						border = ceil(border);
						}else
						{

						}
						}
						*/
					}
					else
					{//yプラス方向に進行

						while (old_border_y < 39)
						{

							border_y = (old_border_x + 1) / slope + center[1];

							for (old_border_y = floor(old_border_y); old_border_y <= floor(border_y); old_border_y++)
							{
								if (old_border_y <= 39 && p_data[(int)old_border_y][(int)old_border_x] != 1)
									p_data[(int)old_border_y][(int)old_border_x] = 2;
							}
							old_border_x++;
							old_border_y = old_border_x / slope + center[1];


						}
					}

				}
				else
				{//xプラス方向に進行

					if (slope < 0)
					{
						while (old_border_x < 19)
						{
							//Sleep(100);
							border_x = (old_border_y - 1 - center[1]) * slope + center[0];

							for (old_border_x = floor(old_border_x); old_border_x <= floor(border_x); old_border_x++)
							{
								if (old_border_x <= 19 && p_data[(int)old_border_y][(int)old_border_x] != 1)
								{
									p_data[(int)old_border_y][(int)old_border_x] = 2;
								}

							}
							old_border_y--;
							old_border_x = (old_border_y - center[1]) * slope + center[0];


						}
					}
					else
					{
						while (old_border_x < 19)
						{
							//Sleep(100);
							border_x = (old_border_y + 1 - center[1]) * slope + center[0];

							for (old_border_x = floor(old_border_x); old_border_x <= floor(border_x); old_border_x++)
							{
								if (old_border_x <= 19 && p_data[(int)old_border_y][(int)old_border_x] != 1)
								{
									p_data[(int)old_border_y][(int)old_border_x] = 2;
								}

							}
							old_border_y++;
							old_border_x = (old_border_y - center[1]) * slope + center[0];


						}
					}
				}

				if (old_p_x != p_x || old_p_y != p_y)
				{
					//printf("(%d,%d)", p_x, p_y);
					//printf("(%lf,%lf,%lf)\n", slope, border_x, border_y);

				}
				old_p_x = p_x;
				old_p_y = p_y;

			}

		}

	}
	printf("\n");
	for (j = 0; j <20; j++)
	{
		for (i = 0; i <40; i++)
		{
			printf("%d,", p_data[i][j]);
		}
	}

}
/*
//左右回避
int MakeAvoidingDirection(int **map, int line, int row){


	int sl = 5;	//探索領域の奥行き
	int sr = 5;	//探索領域の幅(左右片側)

	int center = row / 2;	//領域内の自己位置


	int re = center - sr;	//探索領域の右端
	int le = center + sr;	//探索領域の左端

	int fr = 0;
	for (int i = re; i < center; i++){
		for (int j = 0; j < sl; j++){
			if (map[i][j] != 0)fr = 1;
		}
	}

	int fl = 0;
	for (int i = center; i < le; i++){
		for (int j = 0; j < sl; j++){
			if (map[i][j] != 0)fl = 1;
		}
	}
	printf("fr=%d\tfl=%d\n", fr, fl);
	if (fr == 0 && fl == 0)return 0;		//障害物なし
	else if (fr == 1 && fl == 1)return 1;	//停止
	else if (fr == 1 && fl == 0)return 2;	//左へ回避
	else if (fr == 0 && fl == 1)return 3;	//右へ回避


}
*/
//領域分割
int MakeAvoidingDirection(int **map, int line, int row){

	int center = row / 2;	//領域内の自己位置

	/*二番目*/

	int sl_2 = 10;	//探索領域の奥行き
	int sr_2 = 5;	//探索領域の幅(左右片側)

	int re_2 = center - sr_2;	//探索領域の右端
	int le_2 = center + sr_2;	//探索領域の左端

	bool fr_2 = false;
	for (int i = re_2; i < center; i++){
		for (int j = 0; j < sl_2; j++){
			if (map[i][j] != 0)fr_2 = true;
		}
	}

	bool fl_2 = false;
	for (int i = center; i < le_2; i++){
		for (int j = 0; j < sl_2; j++){
			if (map[i][j] != 0)fl_2 = true;
		}
	}



	/*一番内側*/

	int sl_1 = 5;	//探索領域の奥行き
	int sr_1 = 3;	//探索領域の幅(左右片側)

	int re_1 = center - sr_1;	//探索領域の右端
	int le_1 = center + sr_1;	//探索領域の左端

	bool fr_1 = false;
	for (int i = re_1; i < center; i++){
		for (int j = 0; j < sl_1; j++){
			if (map[i][j] != 0)fr_1 = true;
		}
	}

	bool fl_1 = false;
	for (int i = center; i < le_1; i++){
		for (int j = 0; j < sl_1; j++){
			if (map[i][j] != 0)fl_1 = true;
		}
	}

	/*返り値*/
	if (fr_2 == false && fl_2 == false)return 0;	//障害物なし

	else if (fr_2 == true && fl_2 == false){
		if (fr_1 == false)return 1;					//右遠
		else return 2;								//右近
	}

	else if (fr_2 == false && fl_2 == true){
		if (fl_1 == false)return 3;					//左遠
		else return 4;								//左近
	}


	else{
		if (fr_1 == false && fl_1 == false)return 5;
		else if (fr_1 == true && fl_1 == false)return 6;
		else if (fr_1 == false && fl_1 == true)return 7;
		else return 8;
	}

}
int
main(int argc, char *argv[])
{

	int Polar_Point_data[40][20] = {}, Point_data[40][20] = {};
	int **p_data;
	int x_division_coun = 20, y_division_coun = 40;

	

	urg_t urg;
	int ret;
	long *length_data;
	int length_data_size;
	const int scan_times = 3;
	int i,j;


	p_data = (int **)malloc(sizeof(int *)*y_division_coun);
	for (i = 0; i < y_division_coun; i++)
	{
		p_data[i] = (int *)malloc(sizeof(int *)*x_division_coun);
	}
	for (j = 0; j <20; j++)
	{
		for (i = 0; i <40; i++)
		{
			p_data[i][j]=2;
		}
	}


	// \~japanese "COM1" は、センサが認識されているデバイス名にする必要がある
	const char connect_device[] = "COM13";
	const long connect_baudrate = 115200;

	// \~japanese センサに対して接続を行う。
	ret = urg_open(&urg, URG_SERIAL, connect_device, connect_baudrate);
	// \todo check error code

	// \~japanese データ受信のための領域を確保する
	length_data = (long *)malloc(sizeof(long) * urg_max_data_size(&urg));
	// \todo check length_data is not NULL

	// \~japanese 距離データの計測開始。

	// \todo check error code

	// \~japanese センサから距離データを取得する。

	printf("(%d,%d)", sizeof((p_data[0])) / sizeof(p_data[0][0]), sizeof(p_data) / sizeof(p_data[0]));
	obstacle_detection(p_data, urg);
	printf("%d\n",MakeAvoidingDirection(p_data, x_division_coun, y_division_coun));


	// \todo process length_data array
	long Dis_resolution = 4000 / 40, Rad_resolution = M_PI / 20, Dis_resolution2 = 4000 / 20;

	cv::Mat img = cv::Mat::zeros(800, 800, CV_8UC3);
	cv::Mat cpy_img = cv::Mat::zeros(800, 800, CV_8UC3);


	
	cv::namedWindow("drawing", CV_WINDOW_AUTOSIZE | CV_WINDOW_FREERATIO);

	

	int c = 0;
	int rb_x = 400, rb_y = 400;
	long min_distance;
	long max_distance;
	int p_x, p_y,p_rad,p_leng;
	int ch = img.channels();
	int a;
	//while (1) 
	{
		ret = urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
		length_data_size = urg_get_distance(&urg, length_data, NULL);
		urg_distance_min_max(&urg, &min_distance, &max_distance);
		
		for (i = 0; i < length_data_size; ++i) {

			// \~japanese その距離データのラジアン角度を求め、X, Y の座標値を計算する
			double radian;
			long length;
			int x;
			int y;
			
			radian = urg_index2rad(&urg, i);
			//printf("%lf\n", radian);
			length = length_data[i];
			// \todo check length is valid
			
			
			y = -(long)(length * cos(radian)) / 10 + rb_x;
			x = -(long)(length * sin(radian)) / 10 + rb_y;
			if ((length > min_distance) && (length < max_distance))
			{
				if (radian<M_PI / 2 && radian>-M_PI / 2){
					p_rad = (radian + M_PI / 2) / M_PI*20;
					p_leng = length / Dis_resolution;
					Polar_Point_data[p_leng][p_rad] = 1;
				p_x = (length * cos(radian)) / Dis_resolution2 ;
				p_y = ((length * sin(radian))+4000) / Dis_resolution2;
				Point_data[p_y][p_x]=1;
				
				}
				
				

				cv::line(img, cv::Point(rb_x, rb_y), cv::Point(x, y), cv::Scalar(255, 0, 0), 0, 8);
				if (x >= 0 && x<800 && y >= 0 && y<800){
					a = img.step*y + (x*img.elemSize());
					if (a>0)
					{
						img.data[a + 0] = 255;
						img.data[a + 1] = 255;
						img.data[a + 2] = 0;
					}
				}

			}
			
		}
		cpy_img = img.clone();
		cv::circle(cpy_img, cv::Point(rb_x, rb_y), 30, cv::Scalar(0, 0, 200), 3, 4);

		cv::imshow("drawing", cpy_img);
		cpy_img.release();

		c = cvWaitKey(1);
		if (GetAsyncKeyState('Q'))
		{
			// Esc入力で終了 
			//break;
		}
		else if (GetAsyncKeyState('S'))
		{ // 's'キー入力
			printf("'s'キーが押された\n");

		}
		else if (GetAsyncKeyState(VK_LEFT))
		{
			rb_x -= 10;
		}
		else if (GetAsyncKeyState(VK_UP))
		{
			rb_y -= 10;
		}
		else if (GetAsyncKeyState(VK_RIGHT))
		{
			rb_x += 10;
		}
		else if (GetAsyncKeyState(VK_DOWN))
		{
			rb_y += 10;
		}

	}
	
	
//	Sleep(10000);
	// \~japanese センサとの接続を閉じる。
	
	cv::waitKey(0);
	urg_close(&urg);
	return 0;
	
}




