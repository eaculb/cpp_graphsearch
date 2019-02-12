macx {
	# Silence warnings on Mac
	cache()
}

TEMPLATE = app

# Make sure we do not accidentally #include files placed in 'resources'
CONFIG += no_include_pwd
# Reduce compile times and avoid configuration confusion by excluding Qt libs
CONFIG -= qt
CONFIG -= app_bundle

SOURCES = $$files($$PWD/*.cpp)
SOURCES += $$files($$PWD/StanfordCPPLib/*.cpp)
HEADERS = $$files($$PWD/*.h)
HEADERS += $$files($$PWD/StanfordCPPLib/*.h)

# set up flags for the compiler and Stanford C++ libraries
QMAKE_CXXFLAGS += -std=c++0x \
    -Wall \
    -Wextra \
    -Wreturn-type \
    -Werror=return-type \
    -Wunreachable-code \
    -Wno-sign-compare \
    -g \
    -O0 \
    -DSPL_CONSOLE_WIDTH=750 \
    -DSPL_CONSOLE_HEIGHT=400 \
    -DSPL_CONSOLE_FONTSIZE=16 \
    -DSPL_CONSOLE_ECHO \
    -DSPL_CONSOLE_EXIT_ON_CLOSE \
    -DSPL_CONSOLE_PRINT_EXCEPTIONS \
    -DSPL_CONSOLE_X=999999 \
    -DSPL_CONSOLE_Y=999999 \

INCLUDEPATH += $$PWD/StanfordCPPLib/

# Copies the given files to the destination directory
# The rest of this file defines how to copy the resources folder
defineTest(copyToDestdir) {
    files = $$1

    for(FILE, files) {
        DDIR = $$OUT_PWD

        # Replace slashes in paths with backslashes for Windows
        win32:FILE ~= s,/,\\,g
        win32:DDIR ~= s,/,\\,g

        !win32 {
            copyResources.commands += cp -r '"'$$FILE'"' '"'$$DDIR'"' $$escape_expand(\\n\\t)
        }
        win32 {
            copyResources.commands += xcopy '"'$$FILE'"' '"'$$DDIR'"' /e /y $$escape_expand(\\n\\t)
        }
    }
    export(copyResources.commands)
}

!win32 {
    copyToDestdir($$files($$PWD/resources/*))
}
win32 {
    copyToDestdir($$PWD/resources)
}

copyResources.input = $$files($$PWD/resources/*)
OTHER_FILES = $$files(resources/*)
QMAKE_EXTRA_TARGETS += copyResources
POST_TARGETDEPS += copyResources
