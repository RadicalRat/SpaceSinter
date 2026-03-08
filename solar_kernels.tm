\begindata

    PATH_VALUES  = ( './kernel_data' )
    PATH_SYMBOLS = ( 'KERNELS' )

    KERNELS_TO_LOAD = (
        '$KERNELS/naif0012.tls'
        '$KERNELS/pck00011.tpc'
        '$KERNELS/de442s.bsp'
    )

\begintext

GOLDEN_ENU FRAME DEFINITION
Rotates IAU_EARTH to local East, North, Up at Golden, CO
Lon = -105.22, Lat = 39.75
Sequence: Z-rotation (Lon), Y-rotation (90-Lat), Z-rotation (90 for ENU)

\begindata

    FRAME_GOLDEN_ENU          =  2000001
    FRAME_2000001_NAME        = 'GOLDEN_ENU'
    FRAME_2000001_CLASS       =  4
    FRAME_2000001_CLASS_ID    =  2000001
    FRAME_2000001_CENTER      =  399

    TKFRAME_2000001_RELATIVE  = 'IAU_EARTH'
    TKFRAME_2000001_SPEC      = 'ANGLES'
    TKFRAME_2000001_UNITS     = 'DEGREES'
    TKFRAME_2000001_AXES      = ( 3, 2, 3 )
    TKFRAME_2000001_ANGLES    = ( -105.22, -50.25, 90.0 )

\begintext