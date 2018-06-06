#define DECIMATION_FACTOR       64
#define OUT_FREQ                8000
#define PDM_Input_Buffer_SIZE   ((OUT_FREQ/1000)*DECIMATION_FACTOR/8)
#define PCM_Output_Buffer_SIZE  (OUT_FREQ/1000)
