//////////////////////////////////////////////////////////////////////////////
//
// NotePriorityBuffer
// This class manages MIDI note events as a buffer sorted in priority order
// Priorities: LOW, HIGH, LAST
// Christer Janson (Chrutil) February 2020
//
//////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include "NoteBuffer.h"

MidiNote::MidiNote()
{
    clear();
}

void MidiNote::set(int _note, int _velocity)
{
    note = _note;
    velocity = _velocity;
    time = millis();
}

void MidiNote::clear()
{
    note = 0;
    velocity = 0;
    time = 0;
    tag = 0;
    prev = nullptr;
    next = nullptr;
}

bool MidiNote::compare(MidiNote* that, bool reverse)
{
    if (reverse) return this->note < that->note;
    else return this->note > that->note;
}

#define MAX_NOTE_COUNT 16

NotePriorityBuffer::NotePriorityBuffer() :
    mFirstNote(nullptr)
{
    mFreeList = nullptr;
    mPriority = kHigh;
    mPrevNote = 0;
}

bool NotePriorityBuffer::curNote(int& note, int& velocity, long long& time)
{
    MidiNote* midiNote = mFirstNote;
    if (midiNote)
    {
        note = midiNote->note;
        velocity = midiNote->velocity;
        time = midiNote->time;
        return true;
    }
    return false;   
}

int NotePriorityBuffer::curNote()
{
    MidiNote* midiNote = mFirstNote;
    if (midiNote)
    {
        return midiNote->note;
    }
    return 0;
}

int NotePriorityBuffer::prevNote()
{
    return mPrevNote;
}

void NotePriorityBuffer::setCurNoteTag(int _tag)
{
    MidiNote* midiNote = mFirstNote;
    if (midiNote)
    {
        midiNote->tag = _tag;
    }
}

int NotePriorityBuffer::curNoteTag()
{
    MidiNote* midiNote = mFirstNote;
    if (midiNote)
    {
        return midiNote->tag;
    }
    return 0;
}


void NotePriorityBuffer::setup(Priority p)
{
    mPriority = p;
    mFreeList = allocateFreeList(MAX_NOTE_COUNT);
}

void NotePriorityBuffer::dump(char* buf)
{
    strcpy(buf, ">");
    MidiNote* note = mFirstNote;
    while (note)
    {
        char noteBuf[10];
        itoa(note->note, noteBuf, 10);
        strcat(buf, noteBuf);
        strcat(buf, " ");
        note = note->next;
    }
    strcat(buf, "<");
}

void NotePriorityBuffer::noteOn(int noteVal, int velocity)
{
    mPrevNote = curNote();
    MidiNote* note = allocateNote();
    note->set(noteVal, velocity);
    insertSorted(note);
}

void NotePriorityBuffer::noteOff(int noteVal)
{
    mPrevNote = curNote();
    MidiNote* note = mFirstNote;

    while (note)
    {
        if (note->note == noteVal)
        {
            if (note->prev == nullptr)
            {
                // First note
                mFirstNote = note->next;
                if (mFirstNote)
                    mFirstNote->prev = nullptr;
            }
            else
            {
                MidiNote* p = note->prev;
                MidiNote* n = note->next;

                p->next = n;
                if (n) n->prev = p;
            }

            freeNote(note);
            return;
        }
        note = note->next;
    }

    // Hmm.. this is unexpected, the note was not found.
}

void NotePriorityBuffer::setPriority(Priority p)
{
    mPriority = p;
}

MidiNote* NotePriorityBuffer::allocateNote()
{
    if (mFreeList == nullptr) 
        mFreeList = allocateFreeList(MAX_NOTE_COUNT);

    MidiNote* note = mFreeList;
    mFreeList = mFreeList->next;
    if (mFreeList != nullptr)
        mFreeList->prev = nullptr;
    note->next = nullptr;
    return note;
}

void NotePriorityBuffer::freeNote(MidiNote* note)
{
    note->prev = nullptr;
    note->next = mFreeList;
    if (mFreeList)
        mFreeList->prev = note;
    mFreeList = note;
}

// Freelist is a set of notes allocated for the session
// if more notes are pressed simultaneously, more allocations will
// occur, but no attempt will be made to free the buffers.
MidiNote* NotePriorityBuffer::allocateFreeList(int sz)
{
    MidiNote* newList = new MidiNote[sz];

    newList[0].prev = nullptr;
    newList[sz - 1].next = nullptr;

    // Link up list entries
    for (int i = 0; i < (sz - 1); i++)
    {
        newList[i + 1].prev = &newList[i];
        newList[i].next = &newList[i + 1];
    }

    return newList;
}

void NotePriorityBuffer::insertAfter(MidiNote* first, MidiNote* insert)
{
    if (first == nullptr)
    {
        if (mFirstNote)
            mFirstNote->prev = insert;

        insert->next = mFirstNote;
        insert->prev = nullptr;
        mFirstNote = insert;
    }
    else
    {
        MidiNote* after = first->next;
        insert->next = after;
        if (after) after->prev = insert;
        insert->prev = first;
        first->next = insert;
    }
}

void NotePriorityBuffer::insertSorted(MidiNote* note)
{
    if (mFirstNote == nullptr || mPriority == kLast)
    {
        // Insert note at the beginning of the list
        insertAfter(nullptr, note);
    }
    else  // mPriority == kLow || mPriority == kHigh
    {
        MidiNote* after = nullptr;
        MidiNote* n = mFirstNote;

        while (n)
        {
            if (note->compare(n, mPriority != kHigh))
            {
                // insert here
                insertAfter(after, note);
                return;
            }
            after = n;
            n = n->next;
        }

        // insert last
        insertAfter(after, note);
    }
}
