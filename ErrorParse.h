//
// Created by yc on 2021/9/17.
//

#ifndef _ERRORPARSE_H
#define _ERRORPARSE_H

#include <iostream>

void ErrorParse(unsigned int E_code)
{
    switch (E_code) {
        case 0x7500:
            std::cout << " Communication Error." << std::endl;
            break;
        case 0x7300:
            std::cout << " Sensor Error(CRC)." << std::endl;
            break;
        case 0x2220:
            std::cout << "  Continuous over current" << std::endl;
            break;
        case 0x0000:
//            std::cout << "  No Fault." << std::endl;
            break;
        case 0x3331:
            std::cout << " Field circuit interrupted" << std::endl;
            break;
        default:
            std::cout << " Other Error. (ref to datasheet)" << std::endl;
            break;
    }
}

#endif //_ERRORPARSE_H
