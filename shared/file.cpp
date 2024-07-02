#ifdef _WIN32
#include "windows.h"
#endif

#ifdef __linux__
#include <unistd.h>			//readlink
#include <libgen.h>			//dirname
#include <pwd.h>			//getpwuid
#include <dirent.h>
#include <sys/stat.h>
#endif

#ifdef __APPLE__
#include <sys/stat.h>
#include <libgen.h>
#include <sys/dir.h>
#include <sys/dirent.h>
#include <mach-o/dyld.h>
#endif

#include <fstream>
#include <sstream>
#include <algorithm>
#include "shared/file.h"
#include "shared/output.h"
#include "std_ext.h"

#include <filesystem>

bool DirectoryExists(const std::string& path) {
	std::error_code ec;
	const auto isDir = std::filesystem::is_directory(path,ec);
	if(ec) {
		return false;
	}else{
		return isDir;
	}
}
bool SaveFile(const std::string& filename,const void* p,size_t len) {
	std::string fn=GetFileNameRemap(filename);
	FILE* fp=fopen(fn.c_str(),"wb");
	if(!fp)
		return false;
	size_t res=fwrite(p,1,len,fp);
	fclose(fp);
	return res==len ? true : false;
}
bool SaveFile(const std::string& filename,const std::vector<char>& data) {
	return SaveFile(filename,data.data(),data.size());
}

bool LoadFile(std::vector<char>* data,const std::string& filename) {
	std::string fn=GetFileNameRemap(filename);
#ifdef _WIN32
	FILE* pf=_fsopen(fn.c_str(),"rb",_SH_DENYNO);
#else
	FILE * pf=fopen(fn.c_str(),"rb");
#endif
	bool success=false;
	if(pf) {
		fseek(pf,0,SEEK_END);   // non-portable
		int byteSize=(int)ftell(pf);
		rewind(pf);
		if(byteSize>=0) {
			data->resize(byteSize);
			void* buf=data->data();
			if((int)fread(buf,1,byteSize,pf)==byteSize) {
				success=true;
			}
		}
		fclose(pf);
	}
	return success;
}

std::string LoadFile(const std::string& filename,bool binary) {
	std::string fn=GetFileNameRemap(filename);
	std::string result;
	std::ifstream stream(fn,std::ios::ate|(binary ? std::ios::binary:std::ios_base::openmode(0)));
	if(!stream.is_open()) {
		return result;
	}
	result.reserve(stream.tellg());
	stream.seekg(0,std::ios::beg);
	result.assign((std::istreambuf_iterator<char>(stream)),std::istreambuf_iterator<char>());
	return result;
}

struct FileNameRemap {
	std::string m_alias;
	std::string m_path;
};
int g_remapCount=0;
FileNameRemap g_remapList[16];

void AddFilePathRemap(const char* alias,const char* path) {
	if(g_remapCount==countof(g_remapList))
		FATAL("AddFilePathRemap out of space");
	for(int i=0;i!=g_remapCount;i++) {
		if(g_remapList[i].m_alias==alias) {
			g_remapList[i].m_path=path;
			return;
		}
	}
	g_remapList[g_remapCount].m_alias=alias;
	g_remapList[g_remapCount++].m_path=path;
}
void AddFilePathRemap(const char* alias,const std::string& path) {
	AddFilePathRemap(alias,path.c_str());
}

std::string GetFileNameRemapString(const char* fileName) {
	char buf[512];
	const char* p=fileName;
	char* b=buf;
	while(true) {
		int i=0;
		for(;i!=g_remapCount;i++) {
			//uprintf("%d %s->%s",i,g_remapList[i].m_alias.AsChar(),g_remapList[i].m_path.AsChar());
			if(g_remapList[i].m_alias=="~") {
				if(p==fileName && !strncmp(p,g_remapList[i].m_alias.c_str(),g_remapList[i].m_alias.size())) {
					b+=g_remapList[i].m_path.copy(b,int((buf+sizeof(buf))-b),0);
					p+=g_remapList[i].m_alias.size();
					break;
				}

			}else
			if(!strncmp(p,g_remapList[i].m_alias.c_str(),g_remapList[i].m_alias.size())) {
				b+=g_remapList[i].m_path.copy(b,int((buf+sizeof(buf))-b),0);
				p+=g_remapList[i].m_alias.size();
				break;
			}
		}
		if(i==g_remapCount) {
			*b=*p++;
			if(*b++==0)
				break;
		}
	}
	return std::string(buf);
}

std::string GetFileNameRemap(const char* fileName) {
	std::string str=GetFileNameRemapString(fileName);
	stdx::ReplaceAll(&str,"\\","/");
	if(str.find('$')!=std::string::npos) {
		str=GetFileNameRemapString(str.c_str());
	}
	if(str.find('$')!=std::string::npos) {
		FATAL("GetFileNameRemap unable to remap path %s",fileName);			//probably missing alias
	}
	return str;
}
std::string GetFileNameRemap(const std::string& fileName) {
	return GetFileNameRemap(fileName.c_str());
}

#ifdef _WIN32
std::string GetExecutablePath() {
	char p[MAX_PATH];
	HMODULE hModule=GetModuleHandleW(NULL);
	GetModuleFileName(hModule,p,MAX_PATH);
	std::string path;
	path=p;
	stdx::ReplaceAll(&path,"\\","/");
	path.erase(path.begin()+path.rfind('/')+1,path.end());
	return path;
}
std::string GetExecutableDir() {
	char p[MAX_PATH];
	HMODULE hModule=GetModuleHandleW(NULL);
	GetModuleFileName(hModule,p,MAX_PATH);
	std::string path;
	path=p;
	stdx::ReplaceAll(&path,"\\","/");
	path.erase(path.begin()+path.rfind('/')+1,path.end());
	return path;
}
#endif

#ifdef __linux__
std::string GetExecutableDir() {
	char result[ PATH_MAX ];
	ssize_t count = readlink("/proc/self/exe", result, PATH_MAX-1);
	const char *path;
	if(count != -1) {
		result[count]=0;
		path = dirname(result);
		return std::string(path)+"/";
	}
	return std::string("");
}
std::string GetExecutablePath() {
	char result[ PATH_MAX ];
	ssize_t count = readlink("/proc/self/exe", result, PATH_MAX-1);
	if(count != -1) {
		result[count]=0;
		return std::string(result);
	}
	return std::string("");
}
std::string GetHomeDir() {
	const char* home=getenv("HOME");
	if(!home)
		home = getpwuid(getuid())->pw_dir;
	return std::string(home);
}
#endif

#ifdef __APPLE__
#include <cassert>

std::string GetExecutableDir() {
	std::string exePath = GetExecutablePath();
	std::string result = exePath;
	return std::string(dirname_r(exePath.data(), result.data())) + "/";
}

std::string GetExecutablePath() {
	char buf[PATH_MAX];
	uint32_t bufsize = PATH_MAX;
	int status = _NSGetExecutablePath(buf, &bufsize);
	if(status == 0)
		return std::string(buf);

	assert(status == -1);
	char* tmp = new char[bufsize];
	status = _NSGetExecutablePath(tmp, &bufsize);
	assert(status == 0);
	assert(tmp[bufsize-1] == 0);
	auto result = std::string(tmp);
	delete[] tmp;
	return result;
}

#endif

#ifdef _WIN32
std::string GetHomeDir() {
	return getenv("USERPROFILE");
}
std::string GetTempDirectory() {
	return getenv("TEMP");
}
#endif

//File
File::File() {
}
File::File(const char* pszFileName) {
	SetFileName(pszFileName);
}
File::File(const std::string& fileName) {
	SetFileName(fileName.c_str());
}
void File::SetFileName(const std::string& pszFileName) {
	SetFileName(pszFileName.c_str());
}
void File::SetFileName(const char* pszFileName) {
	m_strFileName=GetFileNameRemap(pszFileName).c_str();
#ifndef _WIN32
	char* path=realpath(m_strFileName.c_str(),NULL); //resolve symlink to real path
	if(path) {
		m_strFileName.assign(path);
		free(path);
	}
#endif
}
File::~File() {}

bool File::Exists() {
	FILE* pFile=fopen(m_strFileName.c_str(),"rb");
    //uprintf("exists %s %p", m_strFileName.c_str(),pFile);
	if(pFile) {
        fclose(pFile);
        return true;
    }
    return false;
}
#ifdef _WIN32
bool File::Read(std::vector<char>* buf)const {
	FILE* pf=_fsopen(m_strFileName.c_str(),"rb",_SH_DENYNO);
	bool success=false;
	if(pf) {
		fseek(pf,0,SEEK_END);   // non-portable
		int byteSize=(int)ftell(pf);
		rewind(pf);
		if(byteSize>=0) {
			buf->resize(byteSize);
			if((int)fread(&buf->front(),1,byteSize,pf)==byteSize) {
				success=true;
			}
		}
		fclose(pf);
	}
	return success;
}
#else
bool File::Read(std::vector<char>* buf)const {
	FILE* pf=fopen(m_strFileName.c_str(),"rb");
	bool success=false;
	if(pf) {
		fseek(pf,0,SEEK_END);   // non-portable
		int byteSize=(int)ftell(pf);
		rewind(pf);
		if(byteSize>=0) {
			buf->resize(byteSize);
			if((int)fread(&buf->front(),1,byteSize,pf)==byteSize) {
				success=true;
			}
		}
		fclose(pf);
	}
	return success;
}
#endif
bool File::Write(const void* p,int len) {
	FILE* fp=fopen(m_strFileName.c_str(),"wb");
	if(!fp)
		return false;
	int res=(int)fwrite(p,1,len,fp);
	fclose(fp);
	return res==len ? true : false;
}

bool File::Write(const std::vector<char>& buf) {
	return Write((const void*)buf.data(),(int)buf.size());
}

bool File::Append(const void* p,int len) {
	FILE* fp=fopen(m_strFileName.c_str(),"ab");
	if(!fp)
		return false;
	int res=(int)fwrite(p,1,len,fp);
	fclose(fp);
	return res==len ? true : false;
}

bool File::AppendString(const char* p) {
	return Append(p,(int)strlen(p));
}

