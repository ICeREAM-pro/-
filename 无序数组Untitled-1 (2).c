#include <stdio.h>  
#include <stdlib.h> 
#include <time.h>  
  
int main() {  
    int n;  
    int *arr; 
    int i;  
  
   
    srand(time(NULL));  
  
   
    printf("请输入数组的大小（1-100）: ");  
    scanf("%d", &n);  
  
  
    if (n < 1 || n > 100) {  
        printf("无效的大小，数组大小必须在1到100之间。\n");  
        return 1;
    }  
  
 
    arr = (int*)malloc(n * sizeof(int));  
    if (arr == NULL) {  
        printf("内存分配失败。\n");  
        return 2;
    }  
  
 
    for (i = 0; i < n; i++) {  
        arr[i] = rand() % 100; 
    }  
  
  
    printf("生成的无序数组为：\n");  
    for (i = 0; i < n; i++) {  
        printf("%d ", arr[i]);  
    }  
    printf("\n");  
  
    free(arr);  
  
    return 0;  
}