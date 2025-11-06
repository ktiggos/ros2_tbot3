#include <tbot3_teleop/xbox_driver.hpp>

int XboxDriver::map_buttons(__u16 code){
    switch (code)
    {
    case 304:
    return 0;
    break;

    case 305:
    return 1;
    break;

    case 307:
    return 2;
    break;

    case 308:
    return 3;
    break;

    case 310:
    return 4;
    break;

    case 311:
    return 5;
    break;

    case 314:
    return 6;
    break;

    case 315:
    return 7;
    break;

    case 317:
    return 8;
    break;

    case 318:
    return 9;
    break;

    default:
    return -1;
    break;
    }
}

int XboxDriver::map_axes(__u16 code){
    switch (code)
    {
    case 0:
    return 0;
    break;

    case 1:
    return 1;
    break;

    case 2:
    return 2;
    break;

    case 3:
    return 3;
    break;

    case 4:
    return 4;
    break;

    case 5:
    return 5;

    case 16:
    return 6;
    break;

    case 17:
    return 7;
    break;

    default:
    return -1;
    break;
    }
}