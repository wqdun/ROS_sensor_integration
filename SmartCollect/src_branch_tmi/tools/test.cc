#include <stdio.h>
#include <iostream>

using namespace std;
int binary_search(int key,int a[],int n) //自定义函数binary_search()
{
    int low,high,mid,count=0,count1=0;
    low=0;
    high=n-1;
    while(low<high)    //査找范围不为0时执行循环体语句
    {
        count++;    //count记录査找次数
        mid=(low+high)/2;    //求中间位置
        if(key<a[mid])    //key小于中间值时
            high=mid-1;    //确定左子表范围
        else if(key>a[mid])    //key 大于中间值时
            low=mid+1;    //确定右子表范围
        else if(key==a[mid])    //当key等于中间值时，证明查找成功
        {
            printf("查找成功!\n 查找 %d 次!a[%d]=%d",count,mid,key);    //输出査找次数及所査找元素在数组中的位置
            count1++;    //count1记录查找成功次数
            break;
        }
    }
    if(count1==0)    //判断是否查找失敗
        printf("查找失敗!");    //査找失敗输出no found
    return 0;
}
int main()
{
    int i;
    int key = 98;
    int a[] = {11, 13, 18, 28, 39, 56, 69, 89, 98, 122};
    int n = 10;
    binary_search(key,a,n);    //调用自定义函数
    cout << "\n";
    return 0;
}