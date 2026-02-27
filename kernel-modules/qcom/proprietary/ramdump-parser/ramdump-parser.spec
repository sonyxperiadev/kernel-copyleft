Summary: The Ramdump extensions RPM
Name: ramdump-parser
Version: 1.0
Release: 1%{?dist}
Source0: %{name}-%{version}.tar.gz
License: Qualcomm-Technologies-Inc.-Proprietary
Group: Development/Tools

Requires: linux-ramdump-parser-v2

%description
The ramdump-parser extensions RPM for parsing ramdumps

%prep
%setup -q

%install
rm -rf %{buildroot}
mkdir -p %{buildroot}/%{_bindir}/linux-ramdump-parser-v2
cp -fr ../%{name}-%{version} %{buildroot}/%{_bindir}/linux-ramdump-parser-v2/extensions

%files
%{_bindir}/linux-ramdump-parser-v2/extensions*
