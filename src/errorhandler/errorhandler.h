#pragma once

namespace errorHandler
{

    using Indicator = 0;

    void indicateError(const uint8_t errorCode = 0);

    void indicatePostCode(const uint8_t postCode = 0);
    
}