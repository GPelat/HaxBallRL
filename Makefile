BUILD=build

all:
	@echo "Hello Group-5"

.git:
	git init

HaxBallEnv:
	git submodule add git@gitlab.ldv.ei.tum.de:arl21/HaxBallEnv.git

initcommit:
	git add .
	git commit -m "Initial Commit"

initpush: .git HaxBallEnv initcommit
	git push --set-upstream git@gitlab.ldv.ei.tum.de:arl21/Group-5.git master

${BUILD}:
	mkdir ${BUILD}

doc:
	doxygen Doxyfile

compile: ${BUILD} doc
	cd ${BUILD} && cmake -DCMAKE_BUILD_TYPE=Release ..
	$(MAKE) -C ${BUILD} all

clean:
	rm -rf ${BUILD}
	rm -rf doc
