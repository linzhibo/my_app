//
// Created by zhibo on 18-4-17.
//

#ifndef GMAPPING_JAVA_CPP_GETMAP_H
#define GMAPPING_JAVA_CPP_GETMAP_H


#include <msg/GetMapRequest.h>
#include <msg/GetMapResponse.h>


namespace nav_msgs
{

    struct GetMap
    {

        typedef GetMapRequest Request;
        typedef GetMapResponse Response;
        Request request;
        Response response;

        typedef Request RequestType;
        typedef Response ResponseType;

    }; // struct GetMap
} // namespace nav_msgs
#endif //GMAPPING_JAVA_CPP_GETMAP_H
