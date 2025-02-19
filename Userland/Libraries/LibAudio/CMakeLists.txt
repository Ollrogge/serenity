set(SOURCES
    GenericTypes.cpp
    SampleFormats.cpp
    Loader.cpp
    RIFFTypes.cpp
    WavLoader.cpp
    FlacLoader.cpp
    FlacWriter.cpp
    WavWriter.cpp
    Metadata.cpp
    MP3Loader.cpp
    PlaybackStream.cpp
    QOALoader.cpp
    QOATypes.cpp
    UserSampleQueue.cpp
    VorbisComment.cpp
)

if (SERENITYOS)
    list(APPEND SOURCES ConnectionToServer.cpp)
    list(APPEND SOURCES ConnectionToManagerServer.cpp)
    list(APPEND SOURCES PlaybackStreamSerenity.cpp)
    set(GENERATED_SOURCES
        ../../Services/AudioServer/AudioClientEndpoint.h
        ../../Services/AudioServer/AudioServerEndpoint.h
        ../../Services/AudioServer/AudioManagerClientEndpoint.h
        ../../Services/AudioServer/AudioManagerServerEndpoint.h
    )
endif()

if (HAVE_PULSEAUDIO)
    list(APPEND SOURCES
        PlaybackStreamPulseAudio.cpp
        PulseAudioWrappers.cpp
    )
endif()

serenity_lib(LibAudio audio)
target_link_libraries(LibAudio PRIVATE LibCore LibIPC LibThreading LibUnicode LibCrypto)

if (HAVE_PULSEAUDIO)
    target_link_libraries(LibAudio PRIVATE pulse)
    target_compile_definitions(LibAudio PRIVATE HAVE_PULSEAUDIO=1)
endif()

if (APPLE)
    target_sources(LibAudio PRIVATE PlaybackStreamAudioUnit.cpp)

    find_library(AUDIO_UNIT AudioUnit REQUIRED)
    target_link_libraries(LibAudio PRIVATE ${AUDIO_UNIT})
endif()
