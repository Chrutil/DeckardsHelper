//////////////////////////////////////////////////////////////////////////////
//
// NotePriorityBuffer
// This class manages MIDI nodes to allow managing notes in priority order
// (LOW/HIGH/LAST)
// Christer Janson (Chrutil) February 2020
//
//////////////////////////////////////////////////////////////////////////////

struct MidiNote
{
    MidiNote();

    void        set(int _note, int _velocity);
    void        clear();
    bool        compare(MidiNote* that, bool reverse);

    int         note;
    int         velocity;
    int         tag;
    long long   time;

    MidiNote*   prev;
    MidiNote*   next;
};

class NotePriorityBuffer
{
public:
    enum Priority { kLow, kHigh, kLast };
    
public:
    NotePriorityBuffer();
    
    void        setup(Priority p);
    void        noteOn(int noteVal, int velocity);
    void        noteOff(int noteVal);
    void        setPriority(Priority p);
    void        dump(char* buf);

    bool        curNote(int& note, int& velocity, long long& time);
    int         curNote();
    int         prevNote();
    void        setCurNoteTag(int _tag);
    int         curNoteTag();

private:
    MidiNote*   allocateNote();
    void        freeNote(MidiNote* note);
    MidiNote*   allocateFreeList(int sz);
    void        insertAfter(MidiNote* before, MidiNote* note);
    void        insertSorted(MidiNote* note);

private:
    MidiNote*   mFirstNote; // NoteOn list in priority order
    MidiNote*   mFreeList;  // List of free (unused) notes
    Priority    mPriority;
    int         mPrevNote;
};
