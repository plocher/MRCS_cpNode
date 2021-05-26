# Makefile for publishing Arduino Projects to GitHub
# John Plocher, 2019-2021

GITHUBUSER=plocher
COMPANYA=MRCS Sketch
COMPANYE=MRCS Electronics

PROJECT_ROOT=/Users/plocher/Dropbox/eagle/onGitHub

SHORTLICENSE_MIT=MIT License
LICENSE_MIT=This sketch is licensed under the [${SHORTLICENSE_MIT}](https://opensource.org/licenses/MIT)

SHORTLICENSE_CERN=CERN Open Hardware Licence v1.2
LICENSE_CERN=This technical documentation is licensed under the [${SHORTLICENSE_CERN}](http://www.ohwr.org/attachments/2388/cern_ohl_v_1_2.txt)

SHORTLICENSE_CCNCSA=Creative Commons Attribution-NonCommercial-ShareAlike
LICENSE_CCNCSA=This technical documentation is licensed under the [${SHORTLICENSE_CCNCSA}](https://creativecommons.org/licenses/by-nc-sa/3.0/)

USER=$(shell git config --get user.name 2>/dev/null)
INITIALVERSION=1.5
PROJECT=$(shell basename $$PWD)
CURRENTTAG=$(shell git describe --tags 2>/dev/null)
TAG=$(shell git rev-list --tags --max-count=1 2>/dev/null)
XALLTAGS=$(shell for tag in `git rev-list --tags  --date-order 2>/dev/null`; do git describe --tags $$tag 2>/dev/null; done)
ALLTAGS=$(shell git for-each-ref --sort=-refname refs/tags | grep " tag\t" | sed -e "s/^..* tag.refs\/tags\///")
VERSION=$(shell git describe --tags ${TAG} 2>/dev/null)
MAJOR=$(shell LC_ALL=C echo ${VERSION} | cut -c1-1)
MINOR=$(shell LC_ALL=C /usr/bin/printf "%.*f\n" 1 ${VERSION} | sed -e "s/^[0-9][0-9]*\.//")
NEXTMAJOR="$(shell expr ${MAJOR} + 1).0"
NEXTDOTMINOR=$(shell expr ${MINOR} + 1)
NEXTMINOR="${MAJOR}.${NEXTDOTMINOR}"
MESSAGE=$(shell git log ${VERSION}.. --pretty=\"%s\" 2>/dev/null)
EXISTS=$(shell git ls-remote https://github.com/${GITHUBUSER}/${PROJECT} >/dev/null 2>&1 ; echo $$?)

all: help check

#all:  gerbers publish

help:
	@( \
	    echo "Makefile help for Github interactions"; \
	    echo ""; \
	    echo "make init"; \
	    echo "\t\tCreate the various metadata files that need to be edited."; \
	    echo "make create"; \
	    echo "\t\tTurns a Eagle CAD project directory into a git repo that is mirrored on Github."; \
	    echo "\t\tUse once at the beginning of a project."; \
	    echo "\t\tCreates a local git repo, initializes it with desired files, creates a GitHub repo,"; \
	    echo "\t\ttags the content with an initial version number (${INITIALVERSION}) and pushes the"; \
	    echo "\t\tcontent."; \
	    echo ""; \
	    echo "make status"; \
	    echo "\t\tShow the status of your repo"; \
	    echo ""; \
	    echo "make versions"; \
	    echo "\t\tShow a list of tagged versions and their commit info"; \
	    echo ""; \
	    echo "make commit"; \
	    echo "\t\tLocally checkpoints your work in progress.  Does not push content to GitHub."; \
	    echo "\t\tUse this often - whenever you have made changes to your CAD files that you wouldn't want to lose."; \
	    echo ""; \
	    echo "make minor"; \
	    echo "\t\tPublish a minor release to GitHub"; \
	    echo "\t\tCheckpoints your work in progress, increments the minor version number and pushes content to GitHub."; \
	    echo "\t\tThe definition of minor is your own - consider PCB changes without Schematic changes to be minor."; \
	    echo ""; \
	    echo "make major"; \
	    echo "\t\tPublish a major release to GitHub"; \
	    echo "\t\tCheckpoints your work in progress, increments the major version number and pushes content to GitHub."; \
	    echo "\t\tThe definition of major is your own - consider Schematic changes to be major."; \
	    echo ""; \
	    echo "make gerbers"; \
	    echo "\t\tGenerate gerbers and other derived files for fabrication and publishing"; \
	    echo ""; \
	    echo "make publish"; \
	    echo "\t\tCreate Jekyll content and copy it to your local GitHub Pages repo for publishing.  WORK IN PROGRESS"; \
	    echo ""; \
	    echo "make clean"; \
	    echo "\t\tDelete all derived files created by \"make gerbers\""; \
	)

version:
	@echo ${CURRENTTAG}

versions:
	@for tag in ${ALLTAGS}; do git show --quiet  $$tag; done

debug:
	@(\
	    echo "COMPANYE:    ${COMPANYE}"; \
	    echo "COMPANYA:    ${COMPANYA}"; \
	    echo "USER:        ${USER}"; \
	    echo "GITHUBUSER:  ${GITHUBUSER}"; \
	    echo "LICENSE_CERN:     ${LICENSE_CERN}"; \
	    echo "SHORTLICENSE_CERN:${SHORTLICENSE_CERN}"; \
	    echo "LICENSE_CCNCSA:     ${LICENSE_CCNCSA}"; \
	    echo "SHORTLICENSE_CCNCSA:${SHORTLICENSE_CCNCSA}"; \
	    echo "PROJECT:     ${PROJECT}"; \
	    echo "INITIALVERSION:     ${INITIALVERSION}"; \
	    echo "MESSAGE:     ${MESSAGE}"; \
	    echo "EXISTS:      ${EXISTS}"; \
	    echo "CURRENT:     ${CURRENTTAG}"; \
	    echo "TAG:         ${TAG}"; \
	    echo "ALLTAGS:     ${ALLTAGS}"; \
	    echo "XALLTAGS:     ${XALLTAGS}"; \
	    echo "VERSION:     ${VERSION}"; \
	    echo "MAJOR:       ${MAJOR}"; \
	    echo "MINOR:       ${MINOR}"; \
	    echo "NEXTMAJOR:   ${NEXTMAJOR}"; \
	    echo "NEXTMINOR:   ${NEXTMINOR}"; \
	    echo "Git History: "; \
	    git log --pretty=format:"%h %s" --graph | sed -e "s/^/        /g"; \
	)

fixtag:
	git tag -d ${VERSION}; \
	git tag -a ${VERSION} -m ${VERSION}; \
	git push --delete origin ${VERSION}; \
	git push origin ${VERSION}

status:
	@git status
	@echo "Commits since last release (${VERSION}) to GitHub - use  make minor  or  make major  to push a new version, or   make fixtag  to patch."
	@git log ${VERSION}.. --pretty=format:"    %s"
	@echo "History"
	@( \
	    sch=$(PROJECT).sch; \
	    brd=$(PROJECT).brd; \
	    ino=$(PROJECT).ino; \
	    printf "\t%-5s    %-35s %-35s %-35s\n" "Ver" "$$sch" "$$brd" "$$ino"; \
	    printf "\t%-5s    %-35s %-35s %-35s\n" "===" "=================" "=================" "================="; \
	    for t in $(ALLTAGS); do \
		git checkout --quiet $$t; \
		s=`if [ -f $$sch ]; then ls -lh $$sch | head -1 | cut -c30-49; else echo ==; fi`; \
		b=`if [ -f $$brd ]; then ls -lh $$brd | head -1 | cut -c30-49; else echo ==; fi`; \
		c=`if [ -f $$ino ]; then ls -lh $$ino | head -1 | cut -c30-49; else echo ==; fi`; \
		printf "\t%-5s    %-35s %-35s %-35s\n" "$$t" "$$s" "$$b" "$$c"; \
	    done; \
	    git checkout --quiet master; \
	)

# make sure the needed files are in the directory

INFO:
	@(\
		echo "title: ${PROJECT}"; \
		echo "project: ${PROJECT}"; \
		echo "designer: ${USER}"; \
		echo "author: ${USER}"; \
		echo "fabricated: no"; \
		echo "fab_date: "; \
		echo "status: released"; \
		echo "release: yes"; \
		echo "tags: [eagle, SPCoast]"; \
		echo "tags: [eagle, MRCS]"; \
		echo "tags: [arduino, SPCoast]"; \
	) > INFO

DESCRIPTION:
	@ (echo "first line is a description of $(PROJECT)"; echo; echo "the rest of the file is a longer description") > DESCRIPTION

README.md: DESCRIPTION
	@(\
		echo "# ${PROJECT}"; \
		echo "## License: ${SHORTLICENSE_MIT}"; \
		echo "## License: ${SHORTLICENSE_CERN}"; \
		echo "## License: ${SHORTLICENSE_CCNCSA}"; \
		echo ""; \
		cat DESCRIPTION; \
		echo ""; \
	) > README.md

LICENSE.md:
	@(\
		echo "${LICENSE_MIT}"; \
		echo "${LICENSE_CERN}"; \
		echo "${LICENSE_CCNCSA}"; \
	) > LICENSE.md

.gitignore:
	@for d in "../.." "../../.."; do \
	    if [ -f "$$d/Dotgitignore" ]; then \
		cp "$$d/Dotgitignore" .gitignore; \
	    fi; \
	done

FORCE:

init: DESCRIPTION INFO README.md LICENSE.md .gitignore


# Create an initial github repo for this project
create: createE
createA: init
	-@(\
		D=`head -1 DESCRIPTION`; \
		if [ ! -d ".git" ]; then \
			hub init -g; \
		fi; \
		git add .; \
		if [ "${EXISTS}" -ne 0 ]; then \
			(set -x; \
			hub create -d "$(COMPANYA): $$D"; \
			git commit -m "Initial version"; \
			git tag -a ${INITIALVERSION} -m "Initial version"; \
			git push --set-upstream origin master; \
			git push origin ${INITIALVERSION}; \
			); \
		fi; \
	)

createE: init
	-@(\
		D=`head -1 DESCRIPTION`; \
		if [ ! -d ".git" ]; then \
			hub init -g; \
		fi; \
		git add .; \
		if [ "${EXISTS}" -ne 0 ]; then \
			(set -x; \
			hub create -d "$(COMPANYE): $$D"; \
			git commit -m "Initial version"; \
			git tag -a ${INITIALVERSION} -m "Initial version"; \
			git push --set-upstream origin master; \
			git push origin ${INITIALVERSION}; \
			); \
		fi; \
	)

convert:
	@(\
		ddir="$(PROJECT_ROOT)/$(PROJECT)"; \
		if [ ! -d "$$ddir" ]; then \
			(set -x; mkdir $$ddir); \
			(\
				cd $$ddir; \
				ln -s ../../Makefile.github Makefile; \
				ln -s ../../Dotgitignore .gitignore; \
			); \
		fi;  \
		d=''; \
		f=''; \
		for i in `ls -1`; do \
		if [ -d $$i ]; then \
			if [ $$i == '.git' -o $$i == 'Archive' ]; then \
				true; \
			else \
				d="$$d $$i"; \
			fi; \
		elif [ -f $$i ]; then \
			if [ -h $$i -o "$$(echo "$$i" | cut -c1-1)" == "@" -o $$i == 'Makefile' -o $$i == '.uploaded' -o $$i == '.DS_Store' ]; then \
				true; \
			else \
				f="$$f $$i"; \
			fi; \
		fi; \
		done; \
		echo "Directories to visit: $$d"; \
		echo "Files to copy:        $$f"; \
		for i in $$f; do \
			if [ ! -f $$ddir/$$i ]; then cp $$i $$ddir; else echo "Exists: $$ddir/$$i"; fi; \
		done; \
		needcreate=1; \
		for i in $$d; do \
			echo "DIR: $$i"; \
			(\
				cd $$i; \
				pwd; \
				sf=''; \
				for ii in `ls -1`; do \
					if [ -f $$ii ]; then \
						if [ -h $$ii -o "$$(echo "$$ii" | cut -c1-1)" == "@" -o \
							$$ii == 'Makefile' -o \
							$$ii == "$(PROJECT).sch.png" -o \
							$$ii == "$(PROJECT).brd.png" -o \
							$$ii == "$(PROJECT).top.brd.png" -o \
							$$ii == "$(PROJECT).bot.brd.png" -o \
							$$ii == "$(PROJECT).b#1" -o \
							$$ii == "$(PROJECT).b#2" -o \
							$$ii == "$(PROJECT).b#3" -o \
							$$ii == "$(PROJECT).b#4" -o \
							$$ii == "$(PROJECT).b#5" -o \
							$$ii == "$(PROJECT).s#1" -o \
							$$ii == "$(PROJECT).s#2" -o \
							$$ii == "$(PROJECT).s#3" -o \
							$$ii == "$(PROJECT).s#4" -o \
							$$ii == "$(PROJECT).s#5" -o \
							$$ii == "$(PROJECT).info" -o \
							$$ii == "$(PROJECT).GTL" -o \
							$$ii == "$(PROJECT).GBL" -o \
							$$ii == "$(PROJECT).GTS" -o \
							$$ii == "$(PROJECT).GBS" -o \
							$$ii == "$(PROJECT).GBO" -o \
							$$ii == "$(PROJECT).GTO" -o \
							$$ii == "$(PROJECT).GML" -o \
							$$ii == "$(PROJECT).GKO" -o \
							$$ii == "$(PROJECT).TXT" -o \
							$$ii == "$(PROJECT).bom.txt" -o \
							$$ii == "$(PROJECT).bom.md" -o \
							$$ii == "$(PROJECT).bom.wiki" -o \
							$$ii == "$(PROJECT).dpv" -o \
							$$ii == "$(PROJECT).eagle.tar.z" -o \
							$$ii == "$(PROJECT).eagle.zip" -o \
							$$ii == "$(PROJECT).gerbers.tar.z" -o \
							$$ii == "$(PROJECT).gerbers.zip" -o \
							$$ii == "$(PROJECT).parts.csv" -o \
							$$ii == "$(PROJECT).parts.txt" -o \
							$$ii == "$(PROJECT).parts.wiki" -o \
							$$ii == "$(PROJECT).svg" -o \
							$$ii == '.DS_Store' ]; then \
							true; \
						else \
							sf="$$sf $$ii"; \
						fi; \
					fi; \
				done; \
				echo "$$i:  $$sf"; \
				for ii in $$sf; do \
					echo "+ cp $$i/$$ii $$ddir"; \
					cp $$ii $$ddir;\
				done; \
				echo "==== $$i ===="; \
				ls -1 | grep "@"; \
				for xx in DESCRIPTION INFO ; do \
					(echo "====$$xx===="; cat $$ddir/$$xx; ) | sed -e "s/^/    |/"; \
				done; \
				echo "Press return to edit the DESCRIPTION and INFO files, a to accept: \\c"; read line; \
				if [ -z "$$line" ]; then vi $$ddir/DESCRIPTION $$ddir/INFO; fi; \
				D=`head -1 $$ddir/DESCRIPTION`; \
				if [ $$needcreate -eq 1 ]; then \
					(cd $$ddir; \
						if [ ! -d ".git" ]; then \
							hub init -g; \
						fi; \
						if [ "${EXISTS}" -ne 0 ]; then \
							(set -x; \
								hub create -d "$(COMPANY): $$D"; \
							); \
						fi; \
						(set -x; \
							git add .; \
							git commit -m "$$i"; \
							git tag -a "$$i" -m ""$$i""; \
							git push --set-upstream origin master; \
							git push origin "$$i"; \
						); \
					); \
				else \
					(cd $$ddir; \
						set -x;\
						git add .; \
						git commit -a -m "$$i"; \
						git tag -a "$$i" -m "$$i"; \
						git push; \
						git push origin "$$i";\
					); \
				fi; \
				needcreate=0; \
			); \
		done; \
	)

commit:
	git commit -a

push:
	git push

major:
	-git commit -a -m "Version ${NEXTMAJOR}"
	-git tag -a ${NEXTMAJOR} -m "Version ${NEXTMAJOR}"
	-git push
	-git push origin ${NEXTMAJOR}

minor:
	-git commit -a -m "Version ${NEXTMINOR}"
	-git tag -a ${NEXTMINOR} -m "Version ${NEXTMINOR}"
	-git push
	-git push origin ${NEXTMINOR}

publishsketch:
	convert2jekyll -t ino -n "$(PROJECT)" 

publish: gerbers
	convert2jekyll -t eagle -n "$(PROJECT)"
	@if [ -f $(PROJECT).parts.csv ]; then (set -x; grep -v PTH $(PROJECT).parts.csv > $(PROJECT).SMD-parts.csv); fi
	@if [ -f $(PROJECT)_array.parts.csv ]; then (set -x; grep -v PTH $(PROJECT)_array.parts.csv > $(PROJECT)_array.SMD-parts.csv); fi
	@rm -f $(PROJECT).gpi $(PROJECT).dri
	@rm -f $(PROJECT)_array.gpi $(PROJECT)_array.dri

publishall:
	@( \
	    reversed=''; \
		for t in $(ALLTAGS); do \
			reversed="$$t $$reversed"; \
		done; \
	    for t in $$reversed; do \
		  make clean; \
		  git checkout --quiet $$t; \
		  printf "Publishing Version %s\n" "$$t"; \
		  make clean; \
		  make publish; \
	    done; \
	    make clean; \
	    git checkout --quiet master; \
	 )

gerbers:
	eagle2cam --order --stamp "${CURRENTTAG}" -P ${PROJECT} --noarchive --leave 

clean:
	@# Gerber files, files that scripts generate, temp files...
	@for suffix in ""  ".brd" "_array" ; do \
	    f="${PROJECT}$$suffix"; \
	    rm -f $$f.GBL $$f.GBO $$f.GBP $$f.GBS $$f.GTL $$f.GTO $$f.GTP $$f.GTS $$f.GML $$f.GKO $$f.TXT; \
	    rm -f $$f.bom.md $$f.bot.brd.png $$f.brd.png $$f.brd.svg $$f.dpv $$f.dri; \
	    rm -f $$f.eagle.tar.z $$f.gerbers.tar.z $$f.eagle.tar $$f.gerbers.tar $$f.eagle.zip $$f.gerbers.zip; \
	    rm -f $$f.gpi $$f.info $$f.sch.png $$f.top.brd.png $$f.parts.txt $$f.parts.wiki $$f.parts.csv $$f.SMD-parts.csv; \
	    rm -f $$f.l#? $$f.b#? $$f.s#? ; \
	done
	@# temp files
	@rm -f feeders.csv
	@rm -f *.l#?
	@rm -f *_[0-9][0-9].job *_[0-9][0-9].pro *~snap.scr

x:
	@( \
	    sch=$(PROJECT).sch; \
	    brd=$(PROJECT).brd; \
	    printf "\t%-5s    %-35s  %-35s\n" "Ver" "$$sch" "$$brd"; \
	    printf "\t%-5s    %-35s  %-35s\n" "===" "=================" "================="; \
	    for t in $(ALLTAGS); do \
		git checkout --quiet $$t; \
		s=`ls -lh $$sch | cut -c29-49`; \
		b=`ls -lh $$brd | cut -c29-49`; \
		printf "\t%-5s %-35s %-35s\n" "$$t" "$$s" "$$b"; \
	    done; \
	    git checkout --quiet master; \
	)

# xml doesn't like commas in tags (names, packages...)
# EAGLE uses european measures (comma being decimal sep) which confuses things
fixeagle:
	sed -i "" -e "s/2,15\/1,0/2.15\/1.0/g" -e "s/\"E1,8/\"E1.8/g" -e "s/\"E2,5/\"E2.5/g" -e "s/\"E2,8/\"E2.8/g" -e "s/\"E3,5/\"E3.5/g" -e "s/\"E5-10,5/\"E5-10.5/g" -e "s/\"E7,/\"E7./g" -e "s/\"EB22,/\"EB22./g" -e "s/\"E5-8,/\"E5-8./g" -e "s/\"UD-4X5,8/\"UD-4X5.8/g" -e "s/\"UD-5X5,8/\"UD-5X5.8/g" -e "s/\"UD-6,3X5,8/\"UD-6.3X5.8/g" -e "s/\"UD-6,3X7,7/\"UD-6.3X7.7/g"   *sch *brd

