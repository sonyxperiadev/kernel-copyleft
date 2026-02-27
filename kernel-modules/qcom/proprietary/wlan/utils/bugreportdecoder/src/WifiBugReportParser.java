/*
 * Copyright (c) 2016 Qualcomm Atheros, Inc.
 *
 * All Rights Reserved.
 * Qualcomm Atheros Confidential and Proprietary.
 */

import java.util.zip.*;
import java.util.Date;
import java.io.*;
import sun.misc.BASE64Decoder;
import sun.misc.BASE64Encoder;
import java.util.Arrays;
import java.text.SimpleDateFormat;


class WifiBugReportParser {

    final protected static char[] hexArray = "0123456789ABCDEF".toCharArray();
    public static String bytesToHex(byte[] bytes, int length) {
        char[] hexChars = new char[length * 2];
        for ( int j = 0; j < length; j++ ) {
            int v = bytes[j] & 0xFF;
            hexChars[j * 2] = hexArray[v >>> 4];
            hexChars[j * 2 + 1] = hexArray[v & 0x0F];
        }
        return new String(hexChars);
    }


    public static void writeToFile (FileOutputStream writer, byte [] line, int length) {
        // creates a FileWriter Object
        try {
            writer.write(line, 0, length);
        } catch (FileNotFoundException fe) {
            System.out.println("File not found: ");
        } catch (IOException ie) {
            System.out.println("Can not read file: to write");
        }
    }

    public static int deCompressStr (String tempfile, String opfile) {
        try {
            FileInputStream fin = new FileInputStream(tempfile);
            FileOutputStream fout = new FileOutputStream(opfile);
            InflaterInputStream infis = new InflaterInputStream(fin);

            int c = 0;
            while ((c = infis.read()) != -1) {
                fout.write(c);
            }
            infis.close();

            fout.close();
            fin.close();
        } catch (IOException e) {
            System.out.println("IOexception: " +e.toString());
            return -1;
        }
        return 1;
    }

    public static int getDataAndDeCompress (BufferedReader bReader, String file) {
        String data = "";
        BASE64Decoder decoder = new BASE64Decoder();
        try {
            FileOutputStream writer = new FileOutputStream("tempBugRepRBs_");
            while ((data = bReader.readLine()) != null &&
                   !data.equals("") &&
                   !data.equals("--------------------------------------------------------------------")) {
                byte[] compressedBytes = decoder.decodeBuffer(data);
                writer.write(compressedBytes);
            }
            writer.close();

            File filetemp = new File("tempBugRepRBs_");
            if (filetemp.length() > 0)
                return deCompressStr("tempBugRepRBs_", file);
            else
                return 0;
        } catch (IOException ie) {
            System.out.println("Can not read file: decompress");
            return -1;
        }
    }

    public static int getPlainData (BufferedReader bReader, String file) {
        String data = "", timestamp_bkp = "";
        SimpleDateFormat ts_format = new SimpleDateFormat("MM-dd HH:mm:ss.SSS");

        try {
            FileOutputStream writer = new FileOutputStream(file);
            while ((data = bReader.readLine()) != null &&
                   !data.contains(" - Log End ----")) {
                if (!data.equals("") && !data.equals(" ") &&  !data.equals("\n") && !data.equals(" \n")) {
                    //timestamp format : mm-dd hh:mm:ss.SSS
                    try {
                        Date ts = ts_format.parse(data);
                        timestamp_bkp = ts_format.format(ts);
                    } catch (Exception e1) {
                        writer.write(timestamp_bkp.getBytes());
                    }
                    writer.write(data.getBytes());
                    writer.write('\n');
                }
            }
            writer.close();
            return 0;

        } catch (IOException ie) {
            System.out.println("Can not read file: decompress");
            return -1;
        }
    }

    public static void main(String[] args) {
           String PATH = "";
           if (args.length > 0) {
               PATH = args[0];
               try {
                   String dirName= new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss").format(new Date());
                   dirName = "BugReport_"+dirName;
                   new File(dirName).mkdirs();

                   byte [] result = new byte[128*1024*1024];
                   FileInputStream instream = new FileInputStream (new File(PATH));
                   if (instream != null ) {
                       InputStreamReader streamReader = new InputStreamReader(instream);
                       BufferedReader bReader = new BufferedReader(streamReader);
                       String data = "";
                       int length = 0;
                       String BugRepNum = "BugDump";
                       FileOutputStream writer_ce = null;
                       FileOutputStream writer_dr = null;
                       FileOutputStream writer_fw = null;
                       FileOutputStream writer_pps = null;

                       while ((data = bReader.readLine()) != null ) {
                           if (data.contains("Bug dump ")) {
                               BugRepNum = "BugDump" +data.substring(9);
                               new File(dirName+"/"+BugRepNum).mkdirs();
                           } else if (data.contains("Alert dump ")) {
                               BugRepNum = "AlertDump" +data.substring(11);
                               new File(dirName+"/"+BugRepNum).mkdirs();
                           } else if (data.contains("ring-buffer = ")) {
                               if (data.contains("power_events_rb")) {
                                   if ((length = getDataAndDeCompress(bReader, dirName+ "/" +BugRepNum +"/power_events_rb.bin")) > 0) {
                                   }
                               } else if (data.contains("connectivity_events_rb")) {
                                   if ((length = getDataAndDeCompress(bReader, dirName+ "/" +BugRepNum +"/connectivity_events_rb.bin")) > 0) {
                                   }
                               } else if (data.contains("driver_prints_rb")) {
                                   if ((length = getDataAndDeCompress(bReader, dirName+ "/" +BugRepNum +"/driver_prints_rb.bin")) > 0) {
                                       //System.out.println("Driver prints: length : " +length +" Data: " + bytesToHex(result, length));
                                   }
                               } else if (data.contains("firmware_prints_rb")) {
                                   if ((length = getDataAndDeCompress(bReader, dirName+ "/" +BugRepNum +"/firmware_prints.bin")) > 0) {
                                   }
                               } else if (data.contains("pkt_stats_rb")) {
                                   if ((length = getDataAndDeCompress(bReader, dirName+ "/" +BugRepNum +"/pkt_stats_rb.bin")) > 0) {
                                   }
                               }
                           } else if (data.contains("FW Memory dump")) {
                               System.out.println("FW mem dump");
                               getDataAndDeCompress(bReader, dirName+ "/" +BugRepNum +"/fw_mem_dump.bin");
                           } else if (data.contains("WifiQualifiedNetworkSelector - Log Begin")) {
                               System.out.println("WifiQualifiedNetworkSelector");
                               getPlainData(bReader, dirName+ "/" +"/WifiQualifiedNetworkSelector.txt");
                           }
                       }
                       instream.close();
                   }
               } catch (FileNotFoundException fe) {
                   System.out.println("File not found: ");
               } catch (IOException ie) {
                   System.out.println("Can not read file: main");
               }
           } else
               System.out.println("No file to parse");
    }
}
