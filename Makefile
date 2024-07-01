include Makefile.gcc

distclean:
	@make -f Makefile.gcc clean
	@make -f Makefile.mingw clean
	@make -f Makefile.gcc-arm clean
	@make -f Makefile.gcc-aarch64 clean
	@rm -rf release/*