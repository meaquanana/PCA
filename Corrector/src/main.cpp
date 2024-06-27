#include"corrector.h"
using namespace lzz;
using namespace cv;
using namespace std;

int main()
{
    /*
        step1: 初始化Corrector 
            std::unique_ptr<Pca_Correct> corner_corrector = std::make_unique<Pca_Correct>();

        step2: 找灯条和匹配灯条
               threshold和findcontours
               初始化灯条 : auto light = Light();
                          light.init(contours[i])

        step3: 矫正灯条
            corner_corrector->correctCorners(left_light,right_light,gray_img,target_color);
        
        step4:矫正后的角点
            left_light.top & left_light.bottom
            right_light.top & right_light.bottom

    */
    return 0;
}