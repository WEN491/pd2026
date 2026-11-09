.PHONY: test test-radar test-sentry

test: test-radar test-sentry

test-radar:
	cd radar_side && python3 -m unittest discover -s tests -v

test-sentry:
	cd sentry_side && python3 -m unittest discover -s tests -v
