#define CONV1_IM_DIM 5       // Input image dimension (5x5)
#define CONV1_IM_CH 1        // Single input channel
#define CONV1_KER_DIM 2      // Kernel size (1x1)
#define CONV1_PADDING 0      // No padding
#define CONV1_STRIDE 1       // Stride of 1
#define CONV1_OUT_CH 1       // Single output channel
#define CONV1_OUT_DIM 5      // Output dimension (matches input due to 1x1 kernel and no padding)


#define POOL1_KER_DIM 3
#define POOL1_STRIDE 2
#define POOL1_PADDING 0
#define POOL1_OUT_DIM 16

#define IP1_DIM 4*4*32
#define IP1_IM_DIM 4
#define IP1_IM_CH 32
#define IP1_OUT 10
