
%:
	@echo
	@echo Building $@
	@echo
	$(eval BOARD := boards/$@.mk)
	#ifneq ("$(wildcard $(BOARD))","")
	#	@echo "Invalid board"
	#else
		@make -f $(BOARD)
	#endif

all:
	@echo "Specify a board!"

clean:
	rm -r build
