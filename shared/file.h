#pragma once

#include <vector>
#include <string>

bool LoadFile(std::vector<char>* data,const std::string& filename);
bool SaveFile(const std::string& filename,const void* p,size_t len);
bool SaveFile(const std::string& filename,const std::vector<char>& data);
bool DirectoryExists(const std::string& path);
std::string LoadFile(const std::string& filename,bool binary);

void AddFilePathRemap(const char* alias,const char* path);
void AddFilePathRemap(const char* alias,const std::string& path);
std::string GetFileNameRemap(const char* fileName);
std::string GetFileNameRemap(const std::string& fileName);

std::string GetExecutablePath();
std::string GetExecutableDir();
std::string GetHomeDir();
std::string GetTempDirectory();

class File {
	public:
		File();
		explicit File(const char* pszFileName);
		explicit File(const std::string& pszFileName);
		~File();
		bool Valid()const{return m_strFileName.size() ? true:false;}
		bool Exists();
		void SetFileName(const char* pszFileName);
		void SetFileName(const std::string& pszFileName);
		const char* GetFileName()const{return m_strFileName.c_str();}
		bool Read(std::vector<char>* buf)const;
		bool Write(const void* p,int len);
		bool Write(const std::vector<char>& buf);
		bool Append(const void* p,int len);
		bool AppendString(const char* p);
	protected:
		std::string m_strFileName;
};
