use autodie;
use strict;
use warnings;
use 5.010;

my @devicetype = ("master", "slave1", "slave2", "slave3");

print "Enter the number of index files created in one capture session: ";
my $num_files = <STDIN>;
chomp $num_files;

exit 0 if ($num_files eq "" or "0");

open(my $fh1, '>', 'capture_log.txt')
    or die "Could not open file capture_log.txt";

my $temp_timestamp = 0;

#Loop count = No of devices
for(my $h = 0; $h< 4; $h++)
{    
    print $fh1 "-----------------------------------------------------\n";
    print $fh1 "                 ".$devicetype[$h]."                 \n";
    print $fh1 "-----------------------------------------------------\n";
    
    
    for(my $j = 0; $j< $num_files; $j++)
    {
        open my $fh, '<:raw', $devicetype[$h].'_000'.$j.'_idx.bin';
        
        print $fh1 "                 INDEX".$j."                           \n";
            
        my $bytes_read = read $fh, my $bytes, 24;
        die 'Got $bytes_read but expected 24' unless $bytes_read == 24;
        
        my ($tag, $version, $flags, $numIndex, , $dataFileSize) = unpack 'V V V V Q', $bytes;
        #print $fh1 "-----------------------------------------------------\n";
        #print $fh1 "                 HEADER                              \n";
        #print $fh1 "-----------------------------------------------------\n";
        #print $fh1 "tag = ".$tag."\n";
        #print $fh1 "version = ".$version."\n";
        print $fh1 "flags = ".$flags."\n";
        print $fh1 "numIndex = ".$numIndex."\n";
        print $fh1 "dataFileSize = ".$dataFileSize."\n";
        
        
        my $temp = 0;
            
        for(my $i = 0; $i< $numIndex; $i++)
        {
            $bytes_read = read $fh, my $bytes, 48;
            die 'Got $bytes_read but expected 48' unless $bytes_read == 48;
            
            my ($tag, $version, $flags, $width, $height, $metaSize1, $metaSize2, $metaSize3, $metaSize4, $size, $timestamp, $offset) = unpack 'v v V v v V V V V V Q Q', $bytes;
            print  $fh1 "-----------------------------------------------------\n";
            print  $fh1 "                 INDEX".$i."                           \n";
            print  $fh1 "-----------------------------------------------------\n";
            #print  $fh1 "tag = ".$tag."\n";
            #print  $fh1 "version = ".$version."\n";
            #print  $fh1 "flags = ".$flags."\n";
            #print  $fh1 "height = ".$height."\n";
            #print  $fh1 "width = ".$width."\n";
            #print  $fh1 "metaSize = ".$metaSize1."\t".$metaSize2."\t".$metaSize3."\t".$metaSize4."\t"."\n";
            #print  $fh1 "size = ".$size."\n";
            print  $fh1 "timestamp = ".$timestamp."\n";
            if(($j != 0) && ($temp == 0))
            {
                #print $fh1 "Timestamp offset between files = ".($timestamp - $temp_timestamp)."\n";
                $temp = 1;
            }
            if($i != 0
            #        && ($timestamp - $temp_timestamp > 120000) 
            )
            {
                #print $fh1 "Timestamp offset = ".($timestamp - $temp_timestamp)."\n";
            }
            $temp_timestamp = $timestamp;
            #print "offset = ".$offset."\n";
            
        }
    }
}
