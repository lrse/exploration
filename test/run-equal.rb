#!/usr/bin/env ruby

$tmapdir = "/home/kulich/work/tmap_lite"
$outdir = $tmapdir + "/out4" # root output directory

config = [
   { "map" => "cave.png", "size" => [25, 20], "pose" =>  [16.0, 8.0, 0.0], "grid" => [ 250, 200] },
   { "map" => "cave.png", "size" => [25, 20], "pose" => [8.0, 8.0, 0.0], "grid" => [ 250, 200] },
   { "map" => "cave.png", "size" => [25, 20], "pose" => [2.0, 16.0, 0.0], "grid" => [ 250, 200] },
   { "map" => "cave.png", "size" => [25, 20], "pose" => [4.0, 4.0, 0.0], "grid" => [ 250, 200] },
   { "map" => "cave.png", "size" => [25, 20], "pose" => [20.0, 16.0, 0.0], "grid" => [ 250, 200] },
   { "map" => "autolab.png", "size" => [35, 30], "pose" => [12.0, 18.0, 0.0], "grid" => [ 350, 300] },
  { "map" => "autolab.png", "size" => [35, 30], "pose" => [4.0, 16.0, 0.0], "grid" => [ 350, 300] },
  { "map" => "autolab.png", "size" => [35, 30], "pose" => [2.0, 12.0, 0.0], "grid" => [ 350, 300] },
  { "map" => "autolab.png", "size" => [35, 30], "pose" => [28.0, 10.0, 0.0], "grid" => [ 350, 300] },
  { "map" => "autolab.png", "size" => [35, 30], "pose" => [22.0, 26.0, 0.0], "grid" => [ 350, 300] },
  ]

method = {
    "greedy" => 0,
    "tsp" => 1,
#    "tsp_plus" => 2,
  }

range = [ 2,3,5 ]

repeat = 10


def getLength(f)
  len = 0
  prev_x = -1000
  prev_y = -1000
  f.each { |line|
           point = line.split(' ')
         x = point[0].to_f
         y = point[1].to_f
         if (prev_x != -1000)
         dx = prev_x -x
         dy = prev_y - y
         len = len + Math.sqrt(dx*dx + dy*dy)
         end
         prev_x = x;
         prev_y = y;
         }
  return len
end



if ( !File.exists?($outdir) )
  Dir.mkdir($outdir)
end

$pid = -1

def runMethod(p, r, method_name,method_index)
  #prepare world file
  template = File.new("one.template")
  worldfile = File.new("exploration.world","w")
  template.each { |line|
    line.gsub!("%POSE%","pose [#{p["pose"][0]} #{p["pose"][1]} 0.0 #{p["pose"][2]}]");
    line.gsub!("%SIZE%","#{p["size"][0]} #{p["size"][1]}");
    line.gsub!("%SIZE_2%","#{p["size"][0]/2.0} #{p["size"][1]/2.0}");
    line.gsub!("%MAP%","#{p["map"]}");
    worldfile.print line
  }
  worldfile.close


#prepare tmap config file
  template = File.new("cfg.template")
  cfgfile = File.new("tmap2.cfg","w")
  template.each { |line|
    line.gsub!("%GRID_WIDTH%","#{p["grid"][0]}");
    line.gsub!("%GRID_HEIGHT%","#{p["grid"][1]}");
    cfgfile.print line
  }
  cfgfile.close


            
            
  dirmap = "#{$outdir}/#{p["map"][0..p["map"].index('.')-1]}"
  if ( !File.exists?(dirmap) )
    Dir.mkdir(dirmap)
  end

     dir2 = "#{dirmap}/range-#{r}"
     if ( !File.exists?(dir2) )
       Dir.mkdir(dir2)
     end
                                 
            
    dirPose = "#{dir2}/#{p["pose"][0]}-#{p["pose"][1]}-#{p["pose"][2]}"
    if ( !File.exists?(dirPose) )
      Dir.mkdir(dirPose)
    end

      dir1 = "#{dirPose}/#{method_name}"
      if ( !File.exists?(dir1) )
        Dir.mkdir(dir1)
      end

      last = 0;
      Dir.foreach(dir1) {|x|
        act = x.to_i()
        if (act>last)
          last = act
        end
      }
      last = last + 1

        if ($pid != -1)
          Process.kill("HUP", $pid)
        end
        system("killall -9 player")
        system("killall -9 tmap")
        sleep(3)
        timer = Thread.new() do
          sleep(1200)
          system("killall -9 player")
          system("killall -9 tmap")
        end

        $pid  = fork do
          Signal.trap("HUP") { puts "Ouch!"; exit }
          system("player", "exploration.cfg")
        end
        sleep(3)
        num = "#{last}".rjust(3,'0')
        dir = "#{dir1}/#{num}"
        actual = Dir.pwd()
        Dir.chdir($tmapdir)
        if ( !File.exists?(dir) )
          Dir.mkdir(dir)
        end
        print "$outdir: #{dir}"
        t = Time.new           
        system("./tmap","cfg/stage/tmap2.cfg", "--robot-out-dir=#{dir}/", "--robot-exploration-goal-selection=#{method_index}", "--robot-laser-max-range=#{r}")
        timer.kill!
        filename = dir + "/path.log"
        if ( File.exists?(filename) )
          f = File.new(filename)
          len = getLength(f)
          f.close         
          cf = File.new(dir+"/config.cfg","w")
          cf.puts "map #{p["map"][0..p["map"].index('.')-1]}"
          cf.puts "pose_x #{p["pose"][0]}"
          cf.puts "pose_y #{p["pose"][1]}"
          cf.puts "pose_phi #{p["pose"][2]}"
          cf.puts "laser_range #{r}"
          cf.puts "method #{method_name}"
          cf.puts "exploration_period 500"
          cf.puts "length #{len}"
          cf.puts "time #{t.strftime("%Y-%m-%d %H:%M:%S")}"
          cf.close
        end           
        Dir.chdir(actual)
end


def checkDir(dirname,pose)
   num = 0;
   d = Dir.new(dirname).each.sort
   total = 0
   length = 0
   d.each {|x|
     filename = dirname + "/" + x + "/path.log"
     if ( File.exists?(filename) )
       f = File.new(filename)
       line = f.readline.chop!
       point = line.split(' ')
       px = point[0].to_f
       py = point[1].to_f
       if (px == pose[0] and py ==  pose[1])
         len = getLength(f)
         num = num + 1
         length = length + len
       else
       end
     else
     end
   }   
#   puts "    #{num} (#{d.size()-2})      len: #{length/num}"
   return num
end


def getCount(config,range, method)
  f = File.new(ARGV[0])
  len = getLength(f)
  puts "length:   #{len}"

end


#create directory structure
def createDirectories(config,range,method)
config.each {|p|
  dirmap = "#{$outdir}/#{p["map"][0..p["map"].index('.')-1]}"
  if ( !File.exists?(dirmap) )
    Dir.mkdir(dirmap)
  end
          
  range.each {|r|
     dir2 = "#{dirmap}/range-#{r}"
     if ( !File.exists?(dir2) )
       Dir.mkdir(dir2)
     end
     dirPose = "#{dir2}/#{p["pose"][0]}-#{p["pose"][1]}-#{p["pose"][2]}"
     if ( !File.exists?(dirPose) )
       Dir.mkdir(dirPose)
     end
     method.each_pair { |method_name,method_index|
       dir1 = "#{dirPose}/#{method_name}"
       if ( !File.exists?(dir1) )
         Dir.mkdir(dir1)
       end
    }
  }
}
end

#createDirectories(config,range,method)

def checkAll(config,range,method)
config.each {|p|
  range.each {|r|
    method.each_pair { |method_name,method_index|
      dir = "#{$outdir}/#{p["map"][0..p["map"].index('.')-1]}/range-#{r}/#{p["pose"][0]}-#{p["pose"][1]}-#{p["pose"][2]}/#{method_name}"
      num = checkDir(dir,p["pose"])
      puts "dir:#{dir}   num:#{num}"
    }
  }
}
end


$process = File.new("process.txt","a")

def runAll(config,range,method,number)
config.each {|p|
  range.each {|r|
    method.each_pair { |method_name,method_index|
      dir = "#{$outdir}/#{p["map"][0..p["map"].index('.')-1]}/range-#{r}/#{p["pose"][0]}-#{p["pose"][1]}-#{p["pose"][2]}/#{method_name}"
      num = checkDir(dir,p["pose"])
      puts "dir:#{dir}   num:#{num}"
      if (num < number)
        $process.puts "#{Time.new}  processing #{dir}     #{num}/#{number}"             
        $process.flush
        runMethod(p, r, method_name, method_index)
      else
        $process.puts "#{Time.new}  done #{dir}     #{num}/#{number}"                     
        $process.flush
      end
    }
  }
}                 
end


30.times {
 runAll(config,range,method,20)
}

20.times {
 runAll(config,range,method,30)
}



#runMethod(config[0], 5, "greedy", method["greedy"])

#num = checkDir("/home/kulich/work/tmap_lite/out4/autolab/range-5/28.0-10.0-0.0/greedy",[28.0,10.0,0.0])


#puts "NUMBER=#{num}"