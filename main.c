#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#define min(x,y) ((x) < (y) ? (x) : (y))

#define BUF_SIZE    500
#define MA4_SIZE    8
#define MA2_SIZE    2
#define HAM_SIZE    8
#define MIN_DIST    8
#define MAX_PEAK    15
FILE *fp;
char file_name[20];
char file_name_txt[20];
char file_name_csv[20];
unsigned char gCRawData[BUF_SIZE][20];
unsigned char gCDiffMeanData[BUF_SIZE][20];
unsigned char gC4pMAData[BUF_SIZE-MA4_SIZE+1][20];
unsigned char gCDiff4pMA[BUF_SIZE-MA4_SIZE][20];
unsigned char gC2pMAData[BUF_SIZE-MA2_SIZE+1][20];
unsigned char gCHammData[BUF_SIZE-HAM_SIZE-MA4_SIZE-2][20];

//const int auw_hamm[31] = {41,276,512,276,41};
int gauw_hamm[31];// = {41,276,512,276,41};
int ghamm_sum=0;
float ghamm[31]={0.08,0.253195,0.64236,0.954446,0.954446,0.64236,0.253195,0.08};
int scaled_hamming_window(float *input, int *output){
    int i =0;
    int sum = 0;
    for(i=0;i<HAM_SIZE;i++){
        output[i] = (int)((512 * input[i]));
        sum += output[i];
    }
    return sum;
}
void diff_from_mean(int *an_x,unsigned int *values,unsigned int avg){
    int i = 0;
    char name[20];
    sprintf(name,"%s_diff_mean.csv",file_name);
    fp = fopen(name,"w"); // read mode
    if( fp == NULL )
    {
          perror("Error while opening the file.\n");
          exit(EXIT_FAILURE);
     }
    for(i=0;i<BUF_SIZE;i++){
        an_x[i] = values[i] - avg;
        sprintf(&gCRawData[i],"(%d,%d)\r\n",i,an_x[i]);
        printf("%s",gCRawData[i]);
        fprintf(fp,"%d,\n",an_x[i]);
    }
    fclose(fp);
}
void four_pt_MA(int *an_x){
    int i = 0;
    char name[25];
    sprintf(name,"%s_4pt_avg.csv",file_name);
    fp = fopen(name,"w"); // read mode
    if( fp == NULL )
    {
          perror("Error while opening the file.\n");
          exit(EXIT_FAILURE);
     }
    for(i=0;i<BUF_SIZE-MA4_SIZE+1;i++){
        an_x[i] += (an_x[i+1] + an_x[i+2]+ an_x[i+3]);
        an_x[i] /= MA4_SIZE;
        sprintf(&gC4pMAData[i],"(%d,%d)\r\n",i,an_x[i]);
        printf("%s",gC4pMAData[i]);
        fprintf(fp,"%d,\n",an_x[i]);
    }
    fclose(fp);
}
void diff_btw_4pt_MA(int * an_dx,int * an_x){
    int i = 0;
    char name[25];
    sprintf(name,"%s_diff_4pt_avg.csv",file_name);
    fp = fopen(name,"w"); // read mode
    if( fp == NULL )
    {
          perror("Error while opening the file.\n");
          exit(EXIT_FAILURE);
     }
    for(i=0;i<BUF_SIZE-MA4_SIZE;i++){
        an_dx[i] = an_x[i+1] - an_x[i];
        sprintf(gCDiff4pMA,"(%d,%d)\r\n",i,an_dx[i]);
        printf("%s",gCDiff4pMA);
        fprintf(fp,"%d,\n",an_dx[i]);
    }
    fclose(fp);
}
void two_pt_MA(int * an_dx){
    int i = 0;
    char name[25];
    sprintf(name,"%s_2pt_avg.csv",file_name);
    fp = fopen(name,"w"); // read mode
    if( fp == NULL )
    {
          perror("Error while opening the file.\n");
          exit(EXIT_FAILURE);
     }
    for(i=0;i< BUF_SIZE - MA2_SIZE + 1;i++){
        an_dx[i] = (an_dx[i] + an_dx[i+1])/2;
        sprintf(gC2pMAData,"(%d,%d)\r\n",i,an_dx[i]);
        printf("%s",gC2pMAData);
        fprintf(fp,"%d,\n",an_dx[i]);
    }
    fclose(fp);
}
void hamming_window(int * an_dx){
    int i = 0,k=0,s=0;
    char name[25];
    sprintf(name,"%s_hamm.csv",file_name);
    fp = fopen(name,"w"); // read mode
    if( fp == NULL )
    {
          perror("Error while opening the file.\n");
          exit(EXIT_FAILURE);
     }
    for(i=0;i<BUF_SIZE-HAM_SIZE-MA4_SIZE-2+1;i++){
        s = 0;
        for(k=i;k<i+HAM_SIZE;k++){
            s -= an_dx[k] * gauw_hamm[k-i];
        }
        an_dx[i] = s/(int)ghamm_sum;
        sprintf(gCHammData,"(%d,%d)\r\n",i,an_dx[i]);
        printf("%s",gCHammData);
        fprintf(fp,"%d,\n",an_dx[i]);
    }
    fclose(fp);
    for(i=0;i<BUF_SIZE-HAM_SIZE-MA4_SIZE-2+1;i++){
        s = 0;
        for(k=i;k<i+HAM_SIZE;k++){
            s -= an_dx[k] * gauw_hamm[k-i];
        }
        an_dx[i] = s/(int)ghamm_sum;
        sprintf(gCHammData,"(%d,%d)\r\n",i,an_dx[i]);
        printf("%s",gCHammData);
    }
}
int threshold_calc(int *an_dx){
    int i=0, n_th1 = 0;
    for(i=0;i<BUF_SIZE-HAM_SIZE;i++){
        n_th1 += (an_dx[i] > 0)? an_dx[i] : ((int)0-an_dx[i]);
        printf("n_th[%d] = %d\r\n",i,n_th1);
    }
    n_th1 /= (BUF_SIZE-HAM_SIZE);
    return n_th1;
}
void maxim_find_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num)
/**
* \brief        Find peaks
* \par          Details
*               Find at most MAX_NUM peaks above MIN_HEIGHT separated by at least MIN_DISTANCE
*
* \retval       None
*/
{
    maxim_peaks_above_min_height( pn_locs, pn_npks, pn_x, n_size, n_min_height );
    maxim_remove_close_peaks( pn_locs, pn_npks, pn_x, n_min_distance );
    *pn_npks = min( *pn_npks, n_max_num );
}

void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *pn_npks, int32_t  *pn_x, int32_t n_size, int32_t n_min_height)
/**
* \brief        Find peaks above n_min_height
* \par          Details
*               Find all peaks above MIN_HEIGHT
*
* \retval       None
*/
{
    int32_t i = 1, n_width;
    *pn_npks = 0;

    while (i < n_size-1){
        if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i-1]){            // find left edge of potential peaks
            n_width = 1;
            while (i+n_width < n_size && pn_x[i] == pn_x[i+n_width])    // find flat peaks
                n_width++;
            if (pn_x[i] > pn_x[i+n_width] && (*pn_npks) < 15 ){                            // find right edge of peaks
                pn_locs[(*pn_npks)++] = i;
                // for flat peaks, peak location is left edge
                i += n_width+1;
            }
            else
                i += n_width;
        }
        else
            i++;
    }
}


void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x,int32_t n_min_distance)
/**
* \brief        Remove peaks
* \par          Details
*               Remove peaks separated by less than MIN_DISTANCE
*
* \retval       None
*/
{

    int32_t i, j, n_old_npks, n_dist;

    /* Order peaks from large to small */
    maxim_sort_indices_descend( pn_x, pn_locs, *pn_npks );

    for ( i = -1; i < *pn_npks; i++ ){
        n_old_npks = *pn_npks;
        *pn_npks = i+1;
        for ( j = i+1; j < n_old_npks; j++ ){
            n_dist =  pn_locs[j] - ( i == -1 ? -1 : pn_locs[i] ); // lag-zero peak of autocorr is at index -1
            if ( n_dist > n_min_distance || n_dist < -n_min_distance )
                pn_locs[(*pn_npks)++] = pn_locs[j];
        }
    }

    // Resort indices longo ascending order
    maxim_sort_ascend( pn_locs, *pn_npks );
}

void maxim_sort_ascend(int32_t *pn_x,int32_t n_size)
/**
* \brief        Sort array
* \par          Details
*               Sort array in ascending order (insertion sort algorithm)
*
* \retval       None
*/
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++) {
        n_temp = pn_x[i];
        for (j = i; j > 0 && n_temp < pn_x[j-1]; j--)
            pn_x[j] = pn_x[j-1];
        pn_x[j] = n_temp;
    }
}

void maxim_sort_indices_descend(int32_t *pn_x, int32_t *pn_indx, int32_t n_size)
/**
* \brief        Sort indices
* \par          Details
*               Sort indices according to descending order (insertion sort algorithm)
*
* \retval       None
*/
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++) {
        n_temp = pn_indx[i];
        for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j-1]]; j--)
            pn_indx[j] = pn_indx[j-1];
        pn_indx[j] = n_temp;
    }
}
void peak_locations(int32_t *pn_locs, int32_t *pn_npks, int32_t  *pn_x){
    int i = 0;
    for(i=0;i<*pn_npks;i++){
        printf("(%d,%d)\r\n",pn_locs[i],pn_x[i]);
    }
}
int main()
{
   static unsigned int sample[BUF_SIZE];
   int32_t n_sp02; //SPO2 value
   int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
   int32_t n_heart_rate;   //heart rate value
   int8_t  ch_hr_valid;
   unsigned int mean = 0;
   static int an_x[BUF_SIZE];
   static int an_dx[BUF_SIZE-MA4_SIZE];
   static int an_dy[BUF_SIZE];
   int32_t pn_locs[100];
   int32_t pn_npks;
   int threshhold;
   int32_t  *pn_x;
   int32_t n_size;
   int32_t n_min_height;
   int32_t k = 0;
   int32_t n_peak_interval_sum = 0;
   //FILE *fp;
   int i =0;
   ghamm_sum = scaled_hamming_window(ghamm,gauw_hamm);
   printf("Enter File Name:\r\n");
   scanf("%s",file_name);
   sprintf(file_name_txt,"%s.txt",file_name);
   printf("Opening %s\r\n",file_name_txt);
  fp = fopen(file_name_txt,"r"); // read mode

   if( fp == NULL )
   {
      perror("Error while opening the file.\n");
      exit(EXIT_FAILURE);
   }

   printf("The contents of the file are :\n");

   for(i=0;i<BUF_SIZE;i++){
    fscanf(fp,"%d",&sample[i]);
    mean += sample[i];
    sprintf(&gCDiffMeanData[i],"(%d,%d)\r\n",i,sample[i]);
    printf("%s",gCDiffMeanData[i]);
   }
   fclose(fp);
   strcpy(file_name_csv,file_name);
   strcat(file_name_csv,".csv");
   fp = fopen(file_name_csv,"w");
   for(i=0;i<BUF_SIZE;i++){
    fprintf(fp,"%d,\n",sample[i]);
   }// read mode
    fclose(fp);
   mean /= BUF_SIZE;
   printf("Mean is %d\r\n",mean);
   diff_from_mean(an_x,sample,mean);
   printf("4 Pt Moving avg:\r\n");
   four_pt_MA(an_x);
   printf("Difference of adjacent 4pt MA values:\r\n");
   diff_btw_4pt_MA(an_dx,an_x);
   printf("2 Pt Moving Average:\r\n");
   two_pt_MA(an_dx);
   printf("Hamming Window:\r\n");
   hamming_window(an_dx);
   threshhold = threshold_calc(an_dx);
   printf("Threshold = %d\r\n",threshhold);
   //maxim_heart_rate_and_oxygen_saturation(sample, BUF_SIZE, sample, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
   maxim_peaks_above_min_height(pn_locs, &pn_npks, an_dx, BUF_SIZE-HAM_SIZE,threshhold );
   printf("Before filtering\r\n");
   peak_locations(pn_locs, &pn_npks, an_dx);
   maxim_remove_close_peaks(pn_locs, &pn_npks, an_dx,MIN_DIST);
   printf("After filtering\r\n");
   peak_locations(pn_locs, &pn_npks, an_dx);
   pn_npks = min( pn_npks, MAX_PEAK );
   printf("Final number of peaks is %d\r\n",pn_npks);
   n_peak_interval_sum =0;
  if (pn_npks>=2){
    for (k=1; k<pn_npks; k++) n_peak_interval_sum += (pn_locs[k] -pn_locs[k -1] ) ;
    n_peak_interval_sum =n_peak_interval_sum/(pn_npks-1);
    n_heart_rate =(int32_t)( (10*60)/ n_peak_interval_sum );
    ch_hr_valid  = 1;
  }
  else  {
    n_heart_rate = -999; // unable to calculate because # of peaks are too small
    ch_hr_valid  = 0;
  }
    printf("BR = %d\r\n",n_heart_rate);
   //fclose(fp);
   return 0;
}
