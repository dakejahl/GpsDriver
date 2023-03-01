all:
# 	@astyle --quiet --options=astylerc *.cpp,*.c,*.hpp,*.h
	@cmake -Bbuild -H.; cmake --build build -j 12
	@size build/gps_driver

clean:
	@rm -rf build
	@echo "All build artifacts removed"

.PHONY: all docker install_skynode install_pi clean
