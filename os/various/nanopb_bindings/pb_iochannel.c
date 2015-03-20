#include "ch.h"
#include "hal.h"

#include "chprintf.h"

#include "pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

static bool buf_write(pb_ostream_t *stream, const uint8_t *buf, size_t count){
	BaseSequentialStream *channel = (BaseSequentialStream *)stream->state;
	
	stream->bytes_written = chSequentialStreamWrite(channel, buf, count);
	
	return true;
}

pb_ostream_t pb_ostream_from_iochannel(BaseSequentialStream *channel){
	pb_ostream_t stream;
	
	stream.callback = &buf_write;
	stream.state = (void *)channel;
	stream.max_size = SIZE_MAX;
	stream.bytes_written = 0;
	
	return stream;
}

static bool buf_read(pb_istream_t *stream, uint8_t *buf, size_t count){
	BaseSequentialStream *channel = (BaseSequentialStream *)stream->state;
	size_t bytes_read;
	
	bytes_read = chSequentialStreamRead(channel, buf, count);

	return bytes_read == count;
}
pb_istream_t pb_istream_from_iochannel(BaseSequentialStream *channel, size_t len){
	pb_istream_t stream;
	
	stream.callback = &buf_read;
	stream.state = (void *)channel;
	stream.bytes_left = len;
	
	return stream;
}

