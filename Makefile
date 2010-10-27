# CCACHE = "ccache g++"
CCACHE = "g++"

all:
	@rm -rf release; \
	mkdir release; \
	(cd release && CXX=$(CCACHE) cmake .. && make test)
