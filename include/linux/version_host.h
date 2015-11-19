/*+=========================================================================== 
  Copyright(c) 2012 FIH Corporation. All Rights Reserved.

  File:
        version_host.h

  Description:
        This header file contains version number definitions of 
        host side cpu image - for SCM and development purpose
        only

  Author:
        Bruce Chiu 2012-11-23
        Jimmy Sung 2012-07-05

============================================================================+*/

#ifndef VERSION_HOST_H
#define VERSION_HOST_H

// HOST version format: v<bsp version>.<platform number>.<branch number>.<build number>
#define VER_HOST_BSP_VERSION     "200810"          //(5 digits, Dec.).
#define VER_HOST_PLATFORM_NUMBER "50"            //(2 digits, Dec.). i.e. 2.0/2.1(Eclair);2.2(Froyo) ...
#define VER_HOST_BRANCH_NUMBER   "00"            //(2 digits, Dec.)
#define VER_HOST_BUILD_NUMBER    "<build_no>"    //(6 digits, Dec.)
#define VER_HOST_GIT_COMMIT      "<git_commit>"  //(40 digits, Hex.)


#endif //#ifndef VERSION_HOST_H

