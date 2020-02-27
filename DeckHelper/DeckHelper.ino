//////////////////////////////////////////////////////////////////////////////
//
// Deckard's Helper 
// Christer Janson (Chrutil) February 2020
//
//////////////////////////////////////////////////////////////////////////////
//
// This is a companion sketch for a piece of hardware that connects MIDI in/out and a 'softpot' slider
// to a Teensy 3.1/3.2 microcontroller
// MIDI is passed through the device and will turn Channel AfterTouch messages into Poly AfterTouch messages for the 
// most recently pressed key.
// There is also a connector for a softpot potentiometer that will act as a cs-80 style ribbon.
// The ribbon code is a bit obscure as it needs lots of filtering to work well, but hopefully not too hard to understand.

// This sketch uses the Arduino MIDI Project from FortySevenEffects on GitHub
// This version has been 'customized' by adding a callback for midi-through so that we can filter
// it before passing the message along.
// The forked and modified MIDI project (called MIDI_CJAT) is renamed so that it does not conflict with the official version.
#include <MIDI_CJAT.h>  
#include "NoteBuffer.h"

// Softpot connection schematics
// Very simple - 3.3v on the high pin, analog ground on the low and the sensor pin in the middle, with a pulldown to ground.
// 
// +3.3v ------------------- [Softpot lower pin]
// 
// A0 -----------|---------- [Softpot center pin]
//              4.7k
// AGND ---------|---------- [Softpot upper pin]
//
// You also need some MIDI in/out circuitry connected to Pin 0 (Rx) and Pin 1 (Tx) on the Teensy
// If you use other pins (or another mcu) you might have to adjust Serial1 below.

// TODO:
// Need an auto calibration routine that can measure the noise level on startup and adjust 'baseline_limit' 
// based on measured levels. For now measure you renvironment and set it manually.
// There shouldn't be any spurious MIDI noise when the softpot is not pressed.
//
// TODO2:
// Just an idea I had. Not sure it's a good one, because the implementation would be a bit messy.
// If we press a new note while applying aftertouch to a previous note, we will currently switch aftertouch to the new note.
// Experiment with keeping aftertouch on a key until there is no more pressure, or the key is released.

USING_NAMESPACE_MIDI

struct AfterTouchMessage
{
    byte note;
    byte pressure;
    byte channel;

    void clear()                      { note = 0; pressure = 0; channel = 0; }
    void set(byte n, byte p, byte c)  { note = n; pressure = p; channel = c; }
};

MIDI_CREATE_INSTANCE_CJAT(HardwareSerial, Serial1, midiIO);
NotePriorityBuffer  midiNoteBuffer;
AfterTouchMessage   lastAfterTouchMessage;
float               baseline = 0;
float               accumulator;
const int           slope_xsize = 20;  // Size of max slope x
const int           slope_ysize = 1;   // Size of max slope y
float               sample[slope_xsize];
float               currentValue;
const int           baseline_limit = 13;
const int           led_pin = 13;
int                 loopCount = 0;
unsigned long       lastMessage = 0;
byte                lastChannelUsed = 0;

void setup()
{
    analogReadResolution(12);
    pinMode(led_pin, OUTPUT);
    
    midiIO.begin(MIDI_CHANNEL_OMNI);
    midiIO.setHandleThru(MIDIThruHandler);
    midiIO.setHandleNoteOn(MIDINoteOnHandler);
    midiIO.setHandleNoteOff(MIDINoteOffHandler);    
    midiIO.turnThruOn(Thru::Full);

    // This manages the last pressed note
    midiNoteBuffer.setup(NotePriorityBuffer::kLast);
    lastAfterTouchMessage.clear();

    // Fill the accumulator with an initial sample
    accumulator = analogRead(A0);
    currentValue = accumulator;
    for (int i = 0; i < slope_xsize; i++)
        sample[i] = accumulator;
}

void loop()
{
    // Fake Poly AT
    // Check MIDI feed, all action is happening in callbacks
    midiIO.read();

    // CS-80 style ribbon
    // The softpot sense pin is connected to A0
    // First sample the ribbon and run a moving average filter on the ribbon sample to smooth out the noise
    float sensorReading = analogRead(A0);
    accumulator = 0.95*accumulator + 0.05*sensorReading;
    
    // Store the sample in a ring buffer
    // This is done so that we can go back 'slope_xsize' samples in time and calculate the current slope.
    // We need to eliminate steep slopes because when pressing or releasing the ribbon the value does not immediately 
    // jump to the location we read, but will ramp up over a few samples.
    sample[loopCount % slope_xsize] = accumulator;
    
    // Get a sample from a few samples back so we can check the slope
    float earlierSample = sample[(loopCount+1) % slope_xsize];

    // Now we run a slope filter, basically filtering out any slopes that are too steep.
    if (abs(accumulator - earlierSample) < slope_ysize)
    {
        currentValue = accumulator;
    }
    else
    {
        // Slope is too steep - ignore sample and keep currentValue
    }

    // Our sample is now filtered.

    // When pressing the ribbon nothing really should happen to the pitch, but we mark the location so that 
    // we can drag or press left or right from here to change the pitch.
    // First press sets the baseline.
    if (baseline < baseline_limit)
    {
        // Our baseline is below the threshold, check the new reading
        if (currentValue > baseline_limit)
        {
            // We put our finger down - Set a new baseline
            digitalWrite(led_pin, HIGH);  // Blip the LED on the Teensy for some visual debugging 
            baseline = currentValue;
            midiIO.sendPitchBend(0, lastChannelUsed);
            digitalWrite(led_pin, LOW);
        }
    }
    else 
    {
        // Baseline is above the threshold so we are pitch-bending 
        if (currentValue <= baseline_limit)
        {
            // The new reading went to below the threshold so we lifted our funger
            digitalWrite(led_pin, HIGH);
            baseline = 0;
            midiIO.sendPitchBend(0, lastChannelUsed);
            digitalWrite(led_pin, LOW);
        }
        else
        {
            // We're pitch bending
            static int lastBend = -1;
            int bend = 2 * (currentValue - baseline);
            if (bend != lastBend)
            {
                // No need to send the same pitch bend again
                if ((millis() - lastMessage) > 10)
                {
                    // Don't send pitch bend messages more often than every 10ms (100 per second should be plenty enough)
                    digitalWrite(led_pin, HIGH);
                    lastMessage = millis();
                    lastBend = bend;
                    midiIO.sendPitchBend(bend, lastChannelUsed);
                    digitalWrite(led_pin, LOW);
                }
            }
        }
    }

    loopCount++;
    if ((loopCount % slope_xsize) == 0)
        loopCount = 0;
}

void MIDINoteOnHandler(byte channel, byte note, byte velocity)
{
    lastChannelUsed = channel;
    // Velocity 0 means note off for some devices - treat it as such
    if (velocity == 0)
    {
        MIDINoteOffHandler(channel, note, velocity);    // Note off
    }
    else
    {
        // Since the note is decoupled from the pressure we have to monitor NoteOn to make sure we clear the pressure
        // of the previous note if there is any.
        if ((lastAfterTouchMessage.note != note) && (lastAfterTouchMessage.pressure != 0))
        {
            // Adding a new note, but we still have pressure on the last key, so clear the pressure.
            midiIO.send(AfterTouchPoly, lastAfterTouchMessage.note, 0, lastAfterTouchMessage.channel);
            lastAfterTouchMessage.clear();
        }
      
        midiNoteBuffer.noteOn(note, velocity);
    }
}

void MIDINoteOffHandler(byte channel, byte note, byte velocity)
{
    lastChannelUsed = channel;
    // Since the note is decoupled from the pressure we have to monitor NoteOff to make sure we're not trying to apply pressure
    // to a note that is no longer pressed.
    if ((lastAfterTouchMessage.note == note) && (lastAfterTouchMessage.pressure != 0))
    {
        // We have pressure on the key we are releasing, clear the pressure.
        midiIO.send(AfterTouchPoly, lastAfterTouchMessage.note, 0, lastAfterTouchMessage.channel);
        lastAfterTouchMessage.clear();
    }
    midiNoteBuffer.noteOff(note);
}

void MIDIThruHandler(MidiType message, byte data1, byte data2, byte channel)
{
    if (message == AfterTouchChannel)
    {
        // Convert Channel AfterTouch to Poly AfterTouch by applying the aftertouch to the most recently pressed key
        byte note = midiNoteBuffer.curNote();
        byte pressure = data1;
        lastAfterTouchMessage.set(note, pressure, channel);
        midiIO.send(AfterTouchPoly, note, pressure, channel);
    }
    else
    {
        midiIO.send(message, data1, data2, channel);
    }
}
