Summary: linux-ramdump-parser-v2 RPM
Name: linux-ramdump-parser-v2
Version: 1.0
Release: 1%{?dist}
Source0: %{name}-%{version}.tar.gz
License: GPL-2.0
Group: Development/Tools

%description
The linux-ramdump-parser-v2 RPM for parsing ramdumps

%prep
%setup -q

%install
rm -rf %{buildroot}
mkdir -p %{buildroot}/%{_bindir}
cp -fr ../%{name}-%{version} %{buildroot}/%{_bindir}/%{name}

%files
%{_bindir}/%{name}*
