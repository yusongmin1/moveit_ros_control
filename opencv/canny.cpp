Mat max_min(Mat image,int max_,int min_)//传入非极大值抑制之后的图片，保留可能边缘
{
    auto img=image.clone();
    int width=image.cols;
    int height=image.rows;
    for(int row=0;row<height;row++)
        for(int col=0;col<width;col++)
        {
            if(image.at<uchar>(row,col)>=max_)img.at<uchar>(row,col)=255;
            else if(image.at<uchar>(row,col)<min_)img.at<uchar>(row,col)=0;
            else 
            {
                for(int i=-1;i<=1;i++)
                    for(int j=-1;j<=1;j++)
                    {
                        if(row+i<0||row+i>height||col+j<0||col+j>width) continue;
                        if(image.at<uchar>(row+i,col+j)>=max_)image.at<uchar>(row,col)=255;
                    }
                if(image.at<uchar>(row,col)!=255)image.at<uchar>(row,col)=0;
            }
        }
}

Mat Non_Maximum_Suppression(Mat x ,Mat Y)//非极大值抑制
{

}



Mat  Canny(Mat image)
{

}

