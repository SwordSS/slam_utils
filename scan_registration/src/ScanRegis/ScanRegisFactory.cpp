#include "ScanRegis/ScanRegisFactory.h"

namespace ScanRegisFactory
{
    extern std::shared_ptr<ScanRegisBase> CreateScanRegisMethod(ScanRegisMode mode)
    {
        switch(mode)
        {
            case PLICP:
                return std::make_shared<ScanRegisWithPLICP>();
        } 
    }

};