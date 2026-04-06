import discord
from discord.ext import commands, tasks
from discord import app_commands
import os

#========================= Constants =========================
#will fill in after to avoid secret being in repo
botToken = '';
#grab token from local env variable
botToken = os.getenv("DISCORD_TOKEN");

GUILD_ID = discord.Object(id=1490450047085838446);

#========================= Global States =========================
embed_door_open = False
door_message = None # discord message
embed_unlocked = False;

#door state to track if door is open or not
    #closed by default
doorState = False;
#door unlocked hardware state
doorUnlockedState = False;


embedColor = discord.Color.red();
embedTitle = "Door State: Locked 🔒"
#discord.Color.green();
#"Door State: Unlocked 🔓"


def build_door_embed():
    global embed_unlocked, embed_door_open
    
    lock_state = "Unlocked 🔓" if embed_unlocked else "Locked 🔒"
    door_state = "Open 🚪" if embed_door_open else "Closed 🚪"
    
    color = discord.Color.green() if embed_unlocked else discord.Color.red()
    
    embed = discord.Embed(
        title=f"Door is {lock_state}",
        description=f"Physical Door State: **{door_state}**",
        color=color
    )
    return embed

 


#========================= Class =========================
class Client(commands.Bot):
    async def on_ready(self):
        print(f'Logged on as {self.user}!');
        try:
            synced = await self.tree.sync(guild=GUILD_ID)
            print(f"Synced {len(synced)} command(s) to guild.")
        except Exception as e:
            print(e)
        
    async def on_message(self, message):
        if message.author == self.user:
            return

        if message.content.startswith('!sesame'):
            await message.channel.send(
                f'Command Recieved: {message.author} executed: {message.content}'
            )

        await self.process_commands(message)
        
        
        
        
    #async background thread to update door message
    @tasks.loop(seconds=5.0) # updates every 5 seconds
    async def hardware_monitor(self):
        global door_message, embed_door_open, embed_unlocked
        if door_message is None:
                    return # door command has not been run yet
        if doorState != embed_door_open or doorUnlockedState != embed_unlocked:
            embed_door_open = doorState;
            embed_unlocked = doorUnlockedState;
            #update variables if any of them are out of sync
            
            try:
                # Edit the saved message directly using the Discord API
                await door_message.edit(embed=build_door_embed(), view=ViewButton())
                print("Discord message dynamically updated from hardware state!")
            except discord.NotFound:
                # The message was deleted in Discord
                door_message = None


 




        

#========================= UI Components =========================
class ViewButton(discord.ui.View):
    
    @discord.ui.button(label="Unlock", style=discord.ButtonStyle.primary, emoji="🧲")
    async def button_callback(self, interaction: discord.Interaction, button: discord.ui.Button):
        global embed_unlocked
        #send door command on button press
        
        #wait 1 s for response
        time.sleep(1);
        #update message
        await interaction.response.edit_message(embed=build_door_embed(), view=self);
 
    
    
    

#========================= Init =========================
intents = discord.Intents.default();
intents.message_content = True;
client = Client(command_prefix = "!sesame", intents = intents);





#========================= Commands =========================
@client.tree.command(name="test", description="test", guild = GUILD_ID)
async def test(interaction: discord.Interaction):
    await interaction.response.send_message("Test");
    
@client.tree.command(name="door", description="Controls Door", guild = GUILD_ID)
async def doorCommand(interaction: discord.Interaction):
    global door_message
    await interaction.response.send_message(embed = build_door_embed(), view=ViewButton());
    door_message = await interaction.original_response()














#run bot
client.run(botToken);
