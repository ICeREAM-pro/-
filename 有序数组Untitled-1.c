#include <stdio.h>  
#include <stdlib.h> 
  

void bubbleSort(int arr[], int n) {  
    int i, j, temp;  
    for (i = 0; i < n-1; i++) {  
        for (j = 0; j < n-i-1; j++) {  
            if (arr[j] > arr[j+1]) {  
                temp = arr[j];  
                arr[j] = arr[j+1];  
                arr[j+1] = temp;  
            }  
        }  
    }  
}  
  

void printArray(int arr[], int size) {  
    int i;  
    for (i = 0; i < size; i++)  
        printf("%d ", arr[i]);  
    printf("\n");  
}  
  
int main() {  
    int n, i;  
    printf("输入数组数量: ");  
    scanf("%d", &n); 
  
    
    int *arr = (int *)malloc(n * sizeof(int));  
    if (arr == NULL) {  
        printf("Memory allocation failed!\n");  
        return 1; 
    }  
  
    
    printf("Enter %d elements:\n", n);  
    for (i = 0; i < n; i++) {  
        scanf("%d", &arr[i]);  
    }  
  
  
    bubbleSort(arr, n);  
  

    printf("Sorted array: \n");  
    printArray(arr, n);  
  
  
    free(arr);  
  
    return 0;  
}