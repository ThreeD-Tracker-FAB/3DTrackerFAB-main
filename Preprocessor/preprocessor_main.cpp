

#include "../common/my_file_io.h"

#include <stdio.h>
#include <sstream>
#include <stdlib.h>
#include <Windows.h>
#include <ShlObj.h>
#include <direct.h>
#include <shlwapi.h>

float voxel_size = 0.01;
bool force_process = false;
bool out_2d_vid = false;

std::vector<bool> cam_enable;

void preProc(const char * data_dir)
{
	int i;

	HANDLE hFind;
	WIN32_FIND_DATAA win32fd;

	char buf[MAX_PATH];
	sprintf(buf, "%s\\*.metadata.xml", data_dir);

	hFind = FindFirstFileA(buf, &win32fd);

	if (hFind == INVALID_HANDLE_VALUE) {
		return;
	}

	sprintf(buf, "%s\\%s", data_dir, win32fd.cFileName);

	printf("%s ... \n", buf);

	MyFileIO fio(buf);

	if (!fio.checkMergedPcDataExist())
	{
		fio.preprosessData(voxel_size, cam_enable);
	}
	else
	{
		printf("preprocess had already been done. \n");

		if (force_process)
		{
			printf("overwriting previous preprocessed data. \n");
			fio.preprosessData(voxel_size, cam_enable);
		}
		else
		{
			printf("the preprocessing was skipped. \n");
		}
	}
	if (out_2d_vid)
	{
		std::vector<bool> vid_exist;
		fio.check2DVideoExist(vid_exist);

		for (i = 0; i < vid_exist.size(); i++)
		{
			if (!vid_exist[i]) break;
		}
		if (i == vid_exist.size())
		{
			printf("2D videos already exist. \n");

			if (force_process)
			{
				printf("overwriting previous 2D videos \n");
				fio.preprosessData(voxel_size);
			}
			else
			{
				printf("the 2D video extraction was skipped. \n");
			}
		}
		else
		{
			fio.export2DVideoFromRGBD();
		}
	}

	FindClose(hFind);
}

void batchPreProc()
{
	//setting for folder open dialog.
	BROWSEINFOA  bi;
	ITEMIDLIST  *idl;
	char        szTmp[MAX_PATH];
	char        path[MAX_PATH];

	bi.hwndOwner = NULL;
	bi.pidlRoot = NULL;
	bi.pszDisplayName = szTmp;
	bi.lpszTitle = "Select data folder";
	bi.ulFlags = BIF_RETURNONLYFSDIRS;
	bi.lpfn = NULL;
	bi.lParam = 0;
	bi.iImage = 0;

	//show the folder open dialog
	idl = SHBrowseForFolderA(&bi);

	SHGetPathFromIDListA(idl, path);

	if (strlen(path) == 0) return;

	// process the same level (in case that user select only one data)
	preProc(path);


	// process one level deeper
	HANDLE hFind;
	WIN32_FIND_DATAA win32fd;

	char buf[MAX_PATH];
	sprintf(buf, "%s\\*", path);

	hFind = FindFirstFileA(buf, &win32fd);

	if (hFind == INVALID_HANDLE_VALUE) {
		return;
	}

	do {
		if (win32fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {

			if (strcmp(win32fd.cFileName, ".") == 0) continue;
			if (strcmp(win32fd.cFileName, "..") == 0) continue;

			sprintf(buf, "%s\\%s", path, win32fd.cFileName);
			preProc(buf);
		}
		else {

		}
	} while (FindNextFileA(hFind, &win32fd));

	FindClose(hFind);
}

int main(int argc, char *argv[])
{
	for (int i = 1; i < argc; i++) {

		if (strcmp(argv[i], "-vsize") == 0)
		{
			i = i + 1;
			sscanf(argv[i], "%f", &voxel_size);
		}
		else if (strcmp(argv[i], "-f") == 0)
		{
			force_process = true;
		}
		else if (strcmp(argv[i], "-2dvid") == 0)
		{
			out_2d_vid = true;
		}

		if (strcmp(argv[i], "-usecam") == 0)
		{
			i = i + 1;

			for (int j = 0; j < strlen(argv[i]); j++)
			{
				if (argv[i][j] == '1')
				{
					cam_enable.push_back(true);
				}
				else
				{
					cam_enable.push_back(false);
				}
			}

		}

	}

	std::cout << std::endl << "------ Preprocessor Settings ------" << std::endl << std::endl;

	std::cout << "voxel size: " << voxel_size << " m (option -vsize)" << std::endl;

	std::cout << "force processing: " << force_process << " (option -f)" << std::endl;

	std::cout << "output 2D videos: " << out_2d_vid << " (option -2dvid)" << std::endl;

	std::cout << std::endl << "------------------------------------" << std::endl << std::endl;


	batchPreProc();

	return 0;
}