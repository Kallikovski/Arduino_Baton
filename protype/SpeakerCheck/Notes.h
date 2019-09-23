
#define NOTE_C5   48
#define NOTE_D5   50
#define NOTE_E5   52
#define NOTE_F5   53
#define NOTE_G5   55
#define NOTE_A5   57
#define NOTE_B5   59
#define NOTE_C6   60
#define NOTE_D6   62
#define NOTE_E6   64
#define NOTE_F6   65
#define NOTE_G6   67
#define NOTE_A6   69
#define NOTE_B6   71

class Notes
{
public:
	int get_Note(int num)
	{
		int note;
		if (num > 0 && num < sizeof(NOTES)/sizeof(int))
		{
			note = NOTES[num];
		}
		else {
			note = -1;
		}
		return note;
	};

private:
	const int NOTES[14] = {
	  NOTE_C5,
	  NOTE_D5,
	  NOTE_E5,
	  NOTE_F5,
	  NOTE_G5,
	  NOTE_A5,
	  NOTE_B5,
	  NOTE_C6,
	  NOTE_D6,
	  NOTE_E6,
	  NOTE_F6,
	  NOTE_G6,
	  NOTE_A6,
	  NOTE_B6
	};
};
