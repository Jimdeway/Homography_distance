#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <fstream> 
#include <string.h> 

//width height 
//像素座標 
int image3[2][2] = {{212, 570},{1022, 633}};
int image5[2][2] = {{338, 499},{904, 539}};
int image10[2][2] = {{465, 432},{789, 453}};
int image20[2][2] = {{543, 388},{717, 399}};
int image30[2][2] = {{574, 373},{694, 381}};
int image40[2][2] = {{591, 365},{682, 371}};
int image50[2][2] = {{601, 361},{675, 367}};

//width height 
//世界座標 
int world3[2][2] = {{-200, 300},{200, 300}};
int world5[2][2] = {{-200, 500},{200, 500}};
int world10[2][2] = {{-200, 1000},{200, 1000}};
int world20[2][2] = {{-200, 2000},{200, 2000}};
int world30[2][2] = {{-200, 3000},{200, 3000}};
int world40[2][2] = {{-200, 4000},{200, 4000}};
int world50[2][2] = {{-200, 5000},{200, 5000}};

//單應性矩陣 
cv::Mat H_3(3, 3, CV_64FC1);
cv::Mat H_inv_3(3, 3, CV_64FC1);
cv::Mat H_5(3, 3, CV_64FC1);
cv::Mat H_inv_5(3, 3, CV_64FC1);
cv::Mat H_10(3, 3, CV_64FC1);
cv::Mat H_inv_10(3, 3, CV_64FC1);
cv::Mat H_20(3, 3, CV_64FC1);
cv::Mat H_inv_20(3, 3, CV_64FC1);
cv::Mat H_30(3, 3, CV_64FC1);
cv::Mat H_inv_30(3, 3, CV_64FC1);
cv::Mat H_40(3, 3, CV_64FC1);
cv::Mat H_inv_40(3, 3, CV_64FC1);

char ctfct_3, ctfct_5, ctfct_10, ctfct_20, ctfct_30, ctfct_40, ctfct_inv_3, ctfct_inv_5, ctfct_inv_10, ctfct_inv_20, ctfct_inv_30, ctfct_inv_40;

//讀入二進制文件
int readbin(int num, bool inv) {
    
    // 創建一個用來指向3X3 cv::Mat的指標 
    double* ptr;

    std::string file_name;

    if(!inv){
        file_name =  "Homography_" + std::to_string(num) + ".bin";
        switch(num){
            case 3:
                ptr = H_3.ptr<double>(); 
                break;
            case 5:
                ptr = H_5.ptr<double>(); 
                break;
            case 10:
                ptr = H_10.ptr<double>(); 
                break;
            case 20:
                ptr = H_20.ptr<double>(); 
                break;
            case 30:
                ptr = H_30.ptr<double>(); 
                break;
            case 40:
                ptr = H_40.ptr<double>(); 
                break;
            default:
                break;
        }
    }    
    else{
        file_name =  "Homography_inv_" + std::to_string(num)+ ".bin";
        switch(num){
            case 3:
                ptr = H_inv_3.ptr<double>(); 
                break;
            case 5:
                ptr = H_inv_5.ptr<double>(); 
                break;
            case 10:
                ptr = H_inv_10.ptr<double>(); 
                break;
            case 20:
                ptr = H_inv_20.ptr<double>(); 
                break;
            case 30:
                ptr = H_inv_30.ptr<double>(); 
                break;
            case 40:
                ptr = H_inv_40.ptr<double>(); 
                break;
            default:
                break;
        }

    }
        
    //打開二進制文件
    std::ifstream inFile(file_name, std::ios::in | std::ios::binary);

    //檢查文件是否打開
    if (!inFile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return 1;
    }

    //獲取文件大小
    inFile.seekg(0, std::ios::end);
    int fileSize = inFile.tellg();
    inFile.seekg(0, std::ios::beg);

    //讀取數據  
	double* data = new double[fileSize / sizeof(double)];
    inFile.read(reinterpret_cast<char*>(data), fileSize);

    //關閉文件
    inFile.close();

     
    

    //將儲存的數據填充到cv::Mat變數中 
	std::memcpy(ptr, data, fileSize);

    //釋放記憶體
    delete[] data;


    return 0;
}

//寫出二進制文件
int writeBin(std::vector<cv::Point2f> src_points,std::vector<cv::Point2f> dst_points ,std::string num ,bool inv) {
    
    cv::Mat H = cv::findHomography(src_points, dst_points);

    cv::Mat H_inv;

    cv::invert(H, H_inv);

    std::string file_name;

    //將矩陣的數據寫入文件
    const double* dataPtr ;

    if(inv){
        file_name =  "Homography_inv_" + num + ".bin";
        //打開二進制文件
        std::ofstream outFile(file_name, std::ios::out | std::ios::binary);

        //檢查文件是否打開
        if (!outFile.is_open()) {
            std::cerr << "Failed to open the file." << std::endl;
            return 1;
        }
        dataPtr = reinterpret_cast<const double*>(H_inv.data);
        outFile.write(reinterpret_cast<const char*>(dataPtr), H_inv.rows * H_inv.cols * sizeof(double));
        //關閉文件
        outFile.close();
    }
        
    else{
        file_name =  "Homography_" + num + ".bin";
        //打開二進制文件
        std::ofstream outFile(file_name, std::ios::out | std::ios::binary);

        //檢查文件是否打開
        if (!outFile.is_open()) {
            std::cerr << "Failed to open the file." << std::endl;
            return 1;
        }
        dataPtr = reinterpret_cast<const double*>(H.data);
        outFile.write(reinterpret_cast<const char*>(dataPtr), H.rows * H.cols * sizeof(double));
        //關閉文件
        outFile.close();    
    }
        
    std::cout << file_name << " has been written" << std::endl;


    return 0;

}


//單應性矩陣讀入(false)與寫出(true)
void Homography(bool mode){
    if(mode){
        std::vector<cv::Point2f> imagePoint3 = {
            cv::Point2f(image3[0][0], image3[0][1]), 
            cv::Point2f(image3[1][0], image3[1][1]), 
            cv::Point2f(image5[0][0], image5[0][1]), 
            cv::Point2f(image5[1][0], image5[1][1])
        };

        std::vector<cv::Point2f> imagePoint5 = {
            cv::Point2f(image5[0][0], image5[0][1]), 
            cv::Point2f(image5[1][0], image5[1][1]), 
            cv::Point2f(image10[0][0], image10[0][1]), 
            cv::Point2f(image10[1][0], image10[1][1])
        };

        std::vector<cv::Point2f> imagePoint10 = {
            cv::Point2f(image10[0][0], image10[0][1]), 
            cv::Point2f(image10[1][0], image10[1][1]), 
            cv::Point2f(image20[0][0], image20[0][1]), 
            cv::Point2f(image20[1][0], image20[1][1])
        };

        std::vector<cv::Point2f> imagePoint20 = {
            cv::Point2f(image20[0][0], image20[0][1]), 
            cv::Point2f(image20[1][0], image20[1][1]), 
            cv::Point2f(image30[0][0], image30[0][1]), 
            cv::Point2f(image30[1][0], image30[1][1])
        };

        std::vector<cv::Point2f> imagePoint30 = {
            cv::Point2f(image30[0][0], image30[0][1]), 
            cv::Point2f(image30[1][0], image30[1][1]), 
            cv::Point2f(image40[0][0], image40[0][1]), 
            cv::Point2f(image40[1][0], image40[1][1])
        };

        std::vector<cv::Point2f> imagePoint40 = {
            cv::Point2f(image40[0][0], image40[0][1]), 
            cv::Point2f(image40[1][0], image40[1][1]), 
            cv::Point2f(image50[0][0], image50[0][1]), 
            cv::Point2f(image50[1][0], image50[1][1])
        };


        std::vector<cv::Point2f> worldPoint3 = {
            cv::Point2f(world3[0][0], world3[0][1]), 
            cv::Point2f(world3[1][0], world3[1][1]), 
            cv::Point2f(world5[0][0], world5[0][1]), 
            cv::Point2f(world5[1][0], world5[1][1])
        };

        std::vector<cv::Point2f> worldPoint5 = {
            cv::Point2f(world5[0][0], world5[0][1]), 
            cv::Point2f(world5[1][0], world5[1][1]), 
            cv::Point2f(world10[0][0], world10[0][1]), 
            cv::Point2f(world10[1][0], world10[1][1])
        };

        std::vector<cv::Point2f> worldPoint10 = {
            cv::Point2f(world10[0][0], world10[0][1]), 
            cv::Point2f(world10[1][0], world10[1][1]), 
            cv::Point2f(world20[0][0], world20[0][1]), 
            cv::Point2f(world20[1][0], world20[1][1])
        };

        std::vector<cv::Point2f> worldPoint20 = {
            cv::Point2f(world20[0][0], world20[0][1]), 
            cv::Point2f(world20[1][0], world20[1][1]), 
            cv::Point2f(world30[0][0], world30[0][1]), 
            cv::Point2f(world30[1][0], world30[1][1])
        };

        std::vector<cv::Point2f> worldPoint30 = {
            cv::Point2f(world30[0][0], world30[0][1]), 
            cv::Point2f(world30[1][0], world30[1][1]), 
            cv::Point2f(world40[0][0], world40[0][1]), 
            cv::Point2f(world40[1][0], world40[1][1])
        };

        std::vector<cv::Point2f> worldPoint40 = {
            cv::Point2f(world40[0][0], world40[0][1]), 
            cv::Point2f(world40[1][0], world40[1][1]), 
            cv::Point2f(world50[0][0], world50[0][1]), 
            cv::Point2f(world50[1][0], world50[1][1])
        };


        writeBin(imagePoint3,worldPoint3,"3",false);
        writeBin(imagePoint3,worldPoint3,"3",true);
        writeBin(imagePoint5,worldPoint5,"5",false);
        writeBin(imagePoint5,worldPoint5,"5",true);
        writeBin(imagePoint10,worldPoint10,"10",false);
        writeBin(imagePoint10,worldPoint10,"10",true);
        writeBin(imagePoint20,worldPoint20,"20",false);
        writeBin(imagePoint20,worldPoint20,"20",true);
        writeBin(imagePoint30,worldPoint30,"30",false);
        writeBin(imagePoint30,worldPoint30,"30",true);
        writeBin(imagePoint40,worldPoint40,"40",false);
        writeBin(imagePoint40,worldPoint40,"40",true);
    }
    else{
        readbin(3, false);
        readbin(3, true);
        readbin(5, false);
        readbin(5, true);
        readbin(10, false);
        readbin(10, true);
        readbin(20, false);
        readbin(20, true);
        readbin(30, false);
        readbin(30, true);
        readbin(40, false);
        readbin(40, true);

    }
    
    
    
    




}

//驗證單應性矩陣的正確性的公式
char Certification_Homography_coordinate_transform(double x,double y, double u_org, double v_org,cv::Mat H, std::string num){
    cv::Mat A = (cv::Mat_<double>(3, 1) << x, y, 1.f);
    cv::Mat tmp = H * A;
    tmp = tmp/tmp.at<double>(2, 0);

    std::string file_name =  "Homography_" + num + ".bin";
    std::cout << "Matrix from " << file_name << ":"<< std::endl;
    std::cout << H << std::endl;
    std::cout << std::endl;
    if(std::fabs(u_org - tmp.at<double>(0, 0)) > 1 || std::fabs(v_org - tmp.at<double>(1, 0)) > 1 ){
        printf("%lf %lf\n",tmp.at<double>(0, 0),tmp.at<double>(1, 0));
        return -1;
    }
    else
        return 0;
}


//驗證單應性矩陣的正確性
int Certification(){
    ctfct_3 = Certification_Homography_coordinate_transform(image5[0][0], image5[0][1],world5[0][0], world5[0][1], H_3, "3");
    ctfct_5 = Certification_Homography_coordinate_transform(image5[0][0], image5[0][1],world5[0][0], world5[0][1], H_5, "5");
    ctfct_10 = Certification_Homography_coordinate_transform(image20[0][0], image20[0][1],world20[0][0], world20[0][1], H_10 ,"10");
    ctfct_20 = Certification_Homography_coordinate_transform(image20[0][0], image20[0][1],world20[0][0], world20[0][1], H_20 ,"20");
    ctfct_30 = Certification_Homography_coordinate_transform(image40[0][0], image40[0][1],world40[0][0], world40[0][1], H_30 ,"30");
    ctfct_40 = Certification_Homography_coordinate_transform(image40[0][0], image40[0][1],world40[0][0], world40[0][1], H_40 ,"40");

    ctfct_inv_3 = Certification_Homography_coordinate_transform(world5[0][0], world5[0][1],image5[0][0], image5[0][1], H_inv_3 ,"inv_3");
    ctfct_inv_5 = Certification_Homography_coordinate_transform(world5[0][0], world5[0][1],image5[0][0], image5[0][1], H_inv_5 ,"inv_5");
    ctfct_inv_10 = Certification_Homography_coordinate_transform(world20[0][0], world20[0][1],image20[0][0], image20[0][1], H_inv_10 ,"inv_10");
    ctfct_inv_20 = Certification_Homography_coordinate_transform(world20[0][0], world20[0][1],image20[0][0], image20[0][1], H_inv_20 ,"inv_20");
    ctfct_inv_30 = Certification_Homography_coordinate_transform(world40[0][0], world40[0][1],image40[0][0], image40[0][1], H_inv_30 ,"inv_30");
    ctfct_inv_40 = Certification_Homography_coordinate_transform(world40[0][0], world40[0][1],image40[0][0], image40[0][1], H_inv_40 ,"inv_40");
    //printf("%d\n",ctfct_3);

    if(ctfct_3 != 0){
        printf("H_3 wrong!\n");
        return -1;
    }
    else if(ctfct_5 != 0){
        printf("H_5 wrong!\n");
        return -1;
    }
    else if(ctfct_10 !=0){
        printf("H_10 wrong!\n");
        return -1;
    }
    else if(ctfct_20 != 0){
        printf("H_20 wrong!\n");
        return -1;
    }
    else if(ctfct_30 != 0){
        printf("H_30 wrong!\n");
        return -1;
    }
    else if(ctfct_40 != 0){
        printf("H_40 wrong!\n");
        return -1;
    }
    else if(ctfct_inv_3 != 0){
        printf("H_inv_3 wrong!\n");
        return -1;
    }
    else if(ctfct_inv_5 != 0){
        printf("H_inv_5 wrong!\n");
        return -1;
    }
    else if(ctfct_inv_10 != 0){
        printf("H_inv_10 wrong!\n");
        return -1;
    }
    else if(ctfct_inv_20 != 0){
        printf("H_inv_20 wrong!\n");
        return -1;
    }
    else if(ctfct_inv_30 != 0){
        printf("H_inv_30 wrong!\n");
        return -1;
    }
    else if(ctfct_inv_40 != 0){
        printf("H_inv_40 wrong!\n");
        return -1;
    }
    else{
        printf("Certification OK!!!\n");
    }
    return 0;

}

void Homography_coordinate_transform(double srcX,double srcY, double &dstX, double &dstY,cv::Mat H){
    cv::Mat A = (cv::Mat_<double>(3, 1) << srcX, srcY, 1.f);
    cv::Mat tmp = H * A;
    tmp = tmp/tmp.at<double>(2, 0);

    dstX = tmp.at<double>(0, 0);
    dstY = tmp.at<double>(1, 0);
}



int main(int argc, char **argv) {
   
    if(argc < 2){
        printf("you need to selet mode, '-c' or '-C'\n");
        return -1;
    }

    
    //計算單應性矩陣並寫出
    if (std::string(argv[1]) == "-w"){
        Homography(true);
        return 0;
    }
	//驗證單應性矩陣的正確性
    else if (std::string(argv[1]) == "-C"){
		//讀取單應性矩陣數據
        Homography(false);
		//驗證單應性矩陣
        Certification();
        return 0;
    }
    else{
        printf("'-d' or '-w' or '-C' \n");
        return -1;
    }

    
   
    return 0;
}
