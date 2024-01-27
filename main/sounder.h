#ifndef sounder_h
#define sounder_h

#ifdef __cplusplus

class Sounder {
	public:
		Sounder();
		void InQueue(unsigned char *buffer);
};

extern "C"

#endif

void SounderInQueue( unsigned char +buffer );


#endif // sounder_h