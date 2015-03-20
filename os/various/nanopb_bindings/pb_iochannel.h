#ifndef _PB_IOCHANNEL_H
#define _PB_IOCHANNEL_H

#include "hal.h"
#include "pb_encode.h"
#include "pb_decode.h"

pb_ostream_t pb_ostream_from_iochannel(BaseSequentialStream *channel);
pb_istream_t pb_istream_from_iochannel(BaseSequentialStream *channel, size_t len);

#endif
