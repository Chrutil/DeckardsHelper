increase_warning_level()

project(midi_CJAT)

add_library(midi_CJAT STATIC
    midi_Namespace.h
    midi_Defs.h
    midi_Message.h
    midi_Settings.h
    midi_RingBuffer.h
    midi_RingBuffer.hpp
    midi_UsbTransport.h
    midi_UsbTransport.hpp
    MIDI_CJAT.cpp
    MIDI_CJAT.hpp
    MIDI_CJAT.h
)
