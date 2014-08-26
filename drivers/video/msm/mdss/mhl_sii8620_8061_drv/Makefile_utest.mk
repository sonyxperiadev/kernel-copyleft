#---------
#
# CppUTest Examples Makefile
#
#----------

#Set this to @ to keep the makefile quiet
ifndef SILENCE
	SILENCE = @
endif

#--- Inputs ----#
export COMPONENT_NAME = tests_component

export CPPUTEST_HOME = CppUTest

export CPPUTEST_USE_EXTENSIONS = Y

export CPP_PLATFORM = Gcc

export CPPUTEST_CPPFLAGS += -DUNIT_TEST

export TEST_SRC_DIRS = \
	./test\
	./test/tests\

export INCLUDE_DIRS =\
  ./\
  ./test\
  ./test/tests\
  ./test/linux_dummy_include\
  $(CPPUTEST_HOME)/include\

mhl_cbus:
	$(MAKE) -f Makefile_cbus

mhl_platform_switch:
	$(MAKE) -f Makefile_platform_switch

# hope this utest can be executed in target device
mhl_platform:
	$(MAKE) -f Makefile_platform

mhl_device:
	$(MAKE) -f Makefile_mhl_device

